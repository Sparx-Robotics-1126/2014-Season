package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.gosparx.IO;

/**
 * The purpose of this class is to implement the drives subsystem.  This class 
 * is in charge of all interaction with the drives systems.
 *
 * @author Justin Bassett (Bassett.JustinT@gmail.com)
 */
public class Drives extends GenericSubsystem {
    
    /**
     * The Drives Class reference
     */
    private static Drives drives;
    
    /**
     * The name of the subsystem for liveWindow
     */
    private final String subsystemName = "Drives";
    
    /**
     * The distance the robot travels per tick of the encoder.
     */
    private static final double DIST_PER_TICK           = 0.0245;
    
    /**
     * The absolute value of the speed at which the motors must be going to 
     * shift.
     */
    private static final double MOTOR_SHIFTING_SPEED    = .15;
    
    /**
     * The position of the solenoid when in low gear.
     */
    private static final boolean LOW_GEAR               = false;
    
    /**
     * The position of the solenoid when in high gear.
     */
    private static final boolean HIGH_GEAR              = true;
    
    /**
     * The speed (inches per second) that we shift up into high gear at.
     */
    private static final double UP_SHIFT_THRESHOLD      = 55;
    
    /**
     * The speed (inches per second) that we shift down into low gear at.
     */
    private static final double DOWN_SHIFT_THRESHOLD    = 25;
    
    /**
     * The time (seconds) we wait for the robot to shift before resuming driver 
     * control.
     */
    private static final double SHIFT_TIME              = .05;
    
    /**
     * The max speed (inches per second) that the robot can obtain.
     */
    public static final double MAX_ROBOT_SPEED         = 168;
    
    private static final int TURNING_THRESHOLD          = 1;
    
    /**
     * This is the speed in inches per second we want the left side of the 
     * drives to achieve.
     */
    private double wantedLeftSpeed;
    
    /**
     * This is controlling the left front drives motor.
     */
    private Talon leftFrontDrives;
    
    /**
     * This is controlling the left rear drives motor.
     */
    private Talon leftRearDrives;
    
    /**
     * This is the encoder on the left side of the robot.
     */
    private Encoder leftDrivesEncoder;
    
    /**
     * This is the speed in inches per second we want the right side of the 
     * drives to achieve.
     */
    private double wantedRightSpeed;
    
    /**
     * This is controlling the right front drives motor.
     */
    private Talon rightFrontDrives;
    
    /**
     * This is controlling the right rear drives motor.
     */
    private Talon rightRearDrives;
    
    /**
     * This is the encoder on the right side of the robot.
     */
    private Encoder rightDrivesEncoder;
    
    /**
     * The time from the {@link Timer#getFPGATimestamp() FPGA Timestamp} that we
     * shifted.
     */
    private double shiftTime;
    
    /**
     * The last time we logged a message to the logger.
     */
    private double lastLogTime = 0;
    
    /**
     * The time between logging messages.
     */
    private static final double LOG_EVERY = 5.0;
    
    /**
     * The solenoid that controls the pneumatics to shift the drives.
     */
    private Solenoid shifter;
    
    private Gyro gyro;
    /**
     * Look to see if there is a drive class, if not it creates one
     * @return the Drives Class 
     */
    public static Drives getInstance(){
        if(drives == null){
            drives = new Drives();
        }
        return drives;
    }
    /**
     * Creates a drives subsystem for controlling the drives subsystem.
     */
    public Drives(){
        super("Drives", Thread.MAX_PRIORITY);
        
        wantedLeftSpeed = 0;
        wantedRightSpeed = 0;
    }

    /**
     * This method gets all of the heavy lifting of setting up all the 
     * components that we plan to control.  See 
     * {@link GenericSubsystem#init() here}.
     */
    public void init() {
        leftFrontDrives = new Talon(IO.DEFAULT_SLOT, IO.LEFT_FRONT_DRIVES_PWM);
        leftRearDrives = new Talon(IO.DEFAULT_SLOT, IO.LEFT_REAR_DRIVES_PWM);
        leftDrivesEncoder = new Encoder(IO.DEFAULT_SLOT, IO.LEFT_DRIVES_ENCODER_CHAN_1,IO.DEFAULT_SLOT,IO.LEFT_DRIVES_ENCODER_CHAN_2, false, EncodingType.k4X);
        leftDrivesEncoder.setDistancePerPulse(DIST_PER_TICK);
        leftDrivesEncoder.start();
        
        rightFrontDrives = new Talon(IO.DEFAULT_SLOT, IO.RIGHT_FRONT_DRIVES_PWM);
        rightRearDrives = new Talon(IO.DEFAULT_SLOT, IO.RIGHT_REAR_DRIVES_PWM);
        rightDrivesEncoder = new Encoder(IO.DEFAULT_SLOT, IO.RIGHT_DRIVES_ENCODER_CHAN_1, IO.DEFAULT_SLOT, IO.RIGHT_DRIVES_ENCODER_CHAN_2, true, EncodingType.k4X);
        rightDrivesEncoder.setDistancePerPulse(DIST_PER_TICK);
        rightDrivesEncoder.start();
        
        shifter = new Solenoid(IO.DEFAULT_SLOT, IO.SHIFT_CHAN);
 
        gyro = new Gyro(IO.GYRO_ANALOG);
        
       LiveWindow.addActuator(subsystemName, "Right Front", rightFrontDrives);
       LiveWindow.addActuator(subsystemName, "Right Rear", rightRearDrives);
       LiveWindow.addSensor(subsystemName, "Right Encoder", rightDrivesEncoder);
       LiveWindow.addActuator(subsystemName, "Left Front", leftFrontDrives);
       LiveWindow.addActuator(subsystemName, "Left Rear", leftRearDrives);
       LiveWindow.addSensor(subsystemName, "Left Encoder", leftDrivesEncoder);
    }

    /**
     * This method takes the speed that is wanted an attempts to match it.
     * 
     * @throws Exception if anything goes wrong!
     */
    public void execute() throws Exception {
        double leftCurrentSpeed, rightCurrentSpeed;
        double leftMotorOutput = 0, rightMotorOutput = 0;
        shiftTime = Timer.getFPGATimestamp();
        
        while(true){
            leftCurrentSpeed = leftDrivesEncoder.getRate();
            rightCurrentSpeed = rightDrivesEncoder.getRate();
            
            leftMotorOutput = getMotorOutput(wantedLeftSpeed, leftCurrentSpeed, leftMotorOutput);
            rightMotorOutput = getMotorOutput(wantedRightSpeed, rightCurrentSpeed, rightMotorOutput);
            
            double averageSpeed = Math.abs((leftCurrentSpeed+rightCurrentSpeed)/2);
            
            if(Timer.getFPGATimestamp() < shiftTime + SHIFT_TIME){
                // if we are shifting don't change the speeds
               leftMotorOutput = leftFrontDrives.get();
               rightMotorOutput = rightFrontDrives.get();
            }else if(averageSpeed >= UP_SHIFT_THRESHOLD && shifter.get() == LOW_GEAR){
                // Shift up
                shifter.set(HIGH_GEAR);
                shiftTime = Timer.getFPGATimestamp();
                leftMotorOutput = MOTOR_SHIFTING_SPEED;
                rightMotorOutput = MOTOR_SHIFTING_SPEED;
            }else if(averageSpeed <= DOWN_SHIFT_THRESHOLD && shifter.get() == HIGH_GEAR){
                // Shift down
                shifter.set(LOW_GEAR);
                shiftTime = Timer.getFPGATimestamp();
                leftMotorOutput = MOTOR_SHIFTING_SPEED;
                rightMotorOutput = MOTOR_SHIFTING_SPEED;
            }
            
            leftFrontDrives.set(leftMotorOutput);//WRONG!!!!
            leftRearDrives.set(leftMotorOutput);
            rightFrontDrives.set(-rightMotorOutput);
            rightRearDrives.set(-rightMotorOutput);
            
            if(Timer.getFPGATimestamp() - LOG_EVERY >= lastLogTime){
                lastLogTime = Timer.getFPGATimestamp();
                log.logMessage("Left: " + wantedLeftSpeed + " Right: " + wantedRightSpeed);
            }
            Thread.sleep(10);
        }
    }
    
    /**
     * This method should act like a PID loop and try to match speeds that are
     * input into the system.
     *
     * @param wantedSpeed the speed wanted in inches per second.
     * @param currentSpeed the current speed in inches per second.
     * @param currentOutput the last speed that was sent to the motors.
     * @return the new speed to set to the motors.
     */
    private double getMotorOutput(double wantedSpeed, double currentSpeed, double currentOutput){
        double speed = 0;
        if(wantedSpeed != 0) {
            // TODO: Make ramping awsomeness!
            speed = ((wantedSpeed - currentSpeed) / MAX_ROBOT_SPEED) + currentOutput;
        }

        return speed;
    }
    
    /**
     * Set the speed of the individual drives in inches per second.
     *
     * @param left speed of left drives system in Inches per Second
     * @param right speed of left drives system in Inches per Second
     */
    public void setSpeed(double left, double right){
        wantedLeftSpeed = left;
        wantedRightSpeed = right;
    }
    
    /**
     * 
     */
    public void turn(double degrees, boolean turnLeft){
        double current = gyro.getAngle();
        double desired = gyro.getAngle() + degrees;
        if(turnLeft)setSpeed(-.4 ,.4);
        if(!turnLeft)setSpeed(.4, -.4);
        boolean done = (((current + TURNING_THRESHOLD >= desired) && !turnLeft) || ((current - TURNING_THRESHOLD <= desired) && turnLeft));
        while(!done){
            current = gyro.getAngle();
            done = (((current + TURNING_THRESHOLD >= desired) && !turnLeft) || ((current - TURNING_THRESHOLD <= desired) && turnLeft));
            try {
                Thread.sleep(10);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }

    public void liveWindow() {
       LiveWindow.addActuator(subsystemName, "Right Front", rightFrontDrives);
       LiveWindow.addActuator(subsystemName, "Right Rear", rightRearDrives);
       LiveWindow.addSensor(subsystemName, "Right Encoder", rightDrivesEncoder);
       LiveWindow.addActuator(subsystemName, "Left Front", leftFrontDrives);
       LiveWindow.addActuator(subsystemName, "Left Rear", leftRearDrives);
       LiveWindow.addSensor(subsystemName, "Left Encoder", leftDrivesEncoder);
    }
}
