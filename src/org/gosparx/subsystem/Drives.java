package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.gosparx.IO;
import org.gosparx.sensors.EncoderData;
import org.gosparx.util.Logger;

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
    private static final double DIST_PER_TICK           = 0.047;//inches
    
    /**
     * The absolute value of the speed at which the motors must be going to 
     * shift in inches per second.
     */
    private static final double MOTOR_SHIFTING_SPEED    = 40;//ips
    
    /**
     * The position of the solenoid when in low gear.
     */
    private static final boolean LOW_GEAR               = false;
    
    /**
     * The speed (inches per second) that we shift up into high gear at.
     */
    private static final double UP_SHIFT_THRESHOLD      = 33;    
    
    /**
     * The speed (inches per second) that we shift down into low gear at.
     */
    private static final double DOWN_SHIFT_THRESHOLD    = 15;//TODO: Check to se if this value has to be so much lower that up_shift_threshold
    
    /**
     * The time (seconds) we wait for the robot to shift before resuming driver 
     * control.
     */
    private static final double SHIFT_TIME              = .1;//MAY NOT BE OPTIMUM IN ALL SITUATION
    
    /**
     * The max speed (inches per second) that the robot can obtain.
     */
    public static final double MAX_ROBOT_SPEED         = 168;
    
    /**
     * The accuracy in degrees for turning 
     */
    private static final double TURNING_THRESHOLD                        =  1;
    
    /**
     * The accuracy in inches for turning
     */
    private static final double DRIVING_THRESHOLD                         = 1;
    
    /**
     * Max degrees that we need to turn, but we still scale.
     */
    private static final double TURNING_MAX = 45;
    
    /**
     * The Y Intercept for the scaling formula.
     */ 
    private static final double Y_INTERCEPT = .5;
    
    /**
     * Max degrees that we need to turn, but we still scale for hold in place.
     */
    private static final double TURNING_MAX_HOLD = 45;
    
    /**
     * The Y Intercept for the scaling formula for hold in place.
     */ 
    private static final double Y_INTERCEPT_HOLD = .25;
    
    /**
     * Number of loops turning must go through to determine accuracy.
     */
    private static final int TURN_COMPLETE_COUNTER = 3;
        
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
     * Used to Calculate Speed and Distance for right side
     */
    private EncoderData rightEncoderData;
    
    /**
     * Used to Calculate Speed and Distance for left side
     */
    private EncoderData leftEncoderData;
    
    /**
     * The time from the {@link Timer#getFPGATimestamp() FPGA Timestamp} that we
     * shifted.
     */
    private double shiftTime;
        
    /**
     * The current angle that the robot is facing. Resets every time turn() is 
     * called
     */ 
    private double currentAngle;
            
    /**
     * The desired angle to turn. Is negative if you re turning left
     */ 
    private double desiredAngle;
    
    /**
     * The relay that the compressor is on.
     */
    private Compressor compressor;
    
    /**
     * The solenoid that controls the pneumatics to shift the drives.
     */
    private Solenoid shifter;
    
    /**
     * The Gyro used for turning calculations
     */
    private Gyro gyro;
    
    /**
     * The gyro analog channel
     */
    private AnalogChannel gyroAnalog;
    
    /**
     * Error to see if the gyro is responding. Used in gyroCheck();
     */
    private static final double GYRO_ERROR = 1;//got this by unplugging encoder and seeing what values it gave

    /**
     * Makes sure that the gyro is functioning
     */
    private boolean isGyroWorking;
    
    /**
     * The current state of the drives. See State class for description and listing of the states.
     */
    private int drivesState;
    
    /**
     * The number of inches to go. Set by driveStraight
     */
    private double inchesToGo;
    
    /**
     * Stores if the drives needs to manually shift to high gear
     */
    private boolean needsToManuallyShiftUp                              = false;
    
    /**
     * Stores if the drives needs to manually shift to low gear 
     */ 
    private boolean needsToManuallyShiftDown                            = false;
    
    /**
     * Stores if we are forcing low gear
     */ 
    private boolean forceLowGear                                        = false;
    
    /**
     * Stores if the drives has disabled autoshifting. Will only shift manually
     * if this is true
     */
    private boolean manualShifting                                      = false;
    
    /**
     * The average distance that the encoders has traveled since the last reset
     */
    private double averageDistEncoder = 0.0;
    
    /**
     * The degrees we still need to go when turning using the 
     * State.FUNCT_TURNING.
     */ 
    private double degToGo;
    
    /**
     * Number of loop that turning has successfully done in threshold
     */
    private int turnLoopCounter = 0;
    
    /**
     * The current state of the autofunctions. Can be any of the State.FUNCT 
     * variables.
     */ 
    private int autoFunctionState;
    
    /**
     * The name of the SmartDashboard variable.
     * Is used to send and get variables from the smartdashboard
     */
    private static final String smartAutoShiftingName = "Auto Shifting";
    
    private double leftMotorOutput = 0; 
    private double rightMotorOutput = 0;
        
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
        super(Logger.SUB_DRIVES, Thread.NORM_PRIORITY);//This may be changed to max 
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
        leftEncoderData = new EncoderData(leftDrivesEncoder, DIST_PER_TICK);
        leftDrivesEncoder.start();
        
        rightFrontDrives = new Talon(IO.DEFAULT_SLOT, IO.RIGHT_FRONT_DRIVES_PWM);
        rightRearDrives = new Talon(IO.DEFAULT_SLOT, IO.RIGHT_REAR_DRIVES_PWM);
        rightDrivesEncoder = new Encoder(IO.DEFAULT_SLOT, IO.RIGHT_DRIVES_ENCODER_CHAN_1, IO.DEFAULT_SLOT, IO.RIGHT_DRIVES_ENCODER_CHAN_2, true, EncodingType.k4X);
        rightDrivesEncoder.setDistancePerPulse(DIST_PER_TICK);
        rightEncoderData = new EncoderData(rightDrivesEncoder, DIST_PER_TICK);
        rightDrivesEncoder.start();
        
        compressor = new Compressor(IO.DEFAULT_SLOT, IO.PRESSURE_SWITCH_CHAN, IO.DEFAULT_SLOT, IO.COMPRESSOR_RELAY_CHAN);
        compressor.start();
        shifter = new Solenoid(IO.DEFAULT_SLOT, IO.SHIFT_CHAN);
        shifter.set(LOW_GEAR);
 
        gyroAnalog = new AnalogChannel(IO.DEFAULT_SLOT, IO.GYRO_ANALOG);
        gyro = new Gyro(gyroAnalog);
        gyro.setSensitivity(0.007);
        isGyroWorking = gyroCheck();
        drivesState = State.DRIVES_LOW_GEAR;
        autoFunctionState = State.FUNCT_STANDBY;
        shiftTime = Timer.getFPGATimestamp();
        leftMotorOutput = 0; 
        rightMotorOutput = 0;
        resetSensors();
    }

    /**
     * This method takes the speed that is wanted an attempts to match it.
     * 
     * @throws Exception if anything goes wrong!
     */
    public void execute() throws Exception {
        double leftCurrentSpeed, rightCurrentSpeed;
        
        currentAngle = gyro.getAngle();
        leftEncoderData.calculateSpeed();
        rightEncoderData.calculateSpeed();
        leftCurrentSpeed = leftEncoderData.getSpeed();
        rightCurrentSpeed = rightEncoderData.getSpeed();
        double averageSpeed = Math.abs((leftCurrentSpeed+rightCurrentSpeed)/2);
        averageDistEncoder = (leftEncoderData.getDistance() + rightEncoderData.getDistance())/2;
        
        switch(autoFunctionState){
            case State.FUNCT_TURNING:
                degToGo = desiredAngle - currentAngle;
                if(isGyroWorking){
                    if(degToGo > 0){
                        leftMotorOutput = (degToGo > TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT)/TURNING_MAX_HOLD)*degToGo+Y_INTERCEPT);
                        rightMotorOutput = -((degToGo > TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT)/TURNING_MAX)*degToGo+Y_INTERCEPT));
                    }else if(degToGo < 0){
                        leftMotorOutput = -((degToGo < -TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT)/TURNING_MAX_HOLD)*degToGo+Y_INTERCEPT));
                        rightMotorOutput = (degToGo < -TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT)/TURNING_MAX_HOLD)*degToGo+Y_INTERCEPT);
                    }
                    if (Math.abs(degToGo) <= TURNING_THRESHOLD && turnLoopCounter == TURN_COMPLETE_COUNTER) {
                        log.logMessage("Done Turning");
                        leftMotorOutput = 0;
                        rightMotorOutput = 0;
                        startHoldPos();
                    }else if(Math.abs(degToGo) < TURNING_THRESHOLD){
                        turnLoopCounter++;
                    }else{
                        turnLoopCounter = 0;
                    }
                    log.logMessage("COMPLETED: " + turnLoopCounter + " Loops || Gyro: "  + currentAngle + " DegToGo: " + degToGo);
                }else{
                    autoFunctionState = State.FUNCT_HOLD_POS;
                } 
                break;
            case State.FUNCT_DRIVE_STRAIGHT:
                leftMotorOutput = getMotorOutput(wantedLeftSpeed, leftCurrentSpeed, leftMotorOutput);
                rightMotorOutput = getMotorOutput(wantedRightSpeed, rightCurrentSpeed, rightMotorOutput);
                if(inchesToGo - leftEncoderData.getDistance() < DRIVING_THRESHOLD){
                    leftMotorOutput = 0;
                }
                if(inchesToGo - rightEncoderData.getDistance() < DRIVING_THRESHOLD){
                    rightMotorOutput = 0;
                }
                if(inchesToGo - rightEncoderData.getDistance() < DRIVING_THRESHOLD && 
                        inchesToGo - leftEncoderData.getDistance() < DRIVING_THRESHOLD){
                    log.logMessage("Done Driving Straight.");
                    setSpeed(0, 0);
                    resetSensors();
                    autoFunctionState = State.FUNCT_STANDBY;
                }
                break;
            case State.FUNCT_HOLD_POS:
                leftMotorOutput = 0;
                rightMotorOutput = 0;
                if(gyro.getAngle() > TURNING_THRESHOLD){
                        leftMotorOutput = -((degToGo < -TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT_HOLD)/TURNING_MAX_HOLD)*degToGo+Y_INTERCEPT_HOLD));
                        rightMotorOutput = (degToGo < -TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT_HOLD)/TURNING_MAX_HOLD)*degToGo+Y_INTERCEPT_HOLD);
                } else if(gyro.getAngle() < -TURNING_THRESHOLD){
                    leftMotorOutput = (degToGo > TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT_HOLD)/TURNING_MAX_HOLD)*degToGo+Y_INTERCEPT_HOLD);
                    rightMotorOutput = -((degToGo > TURNING_MAX_HOLD) ? (1) : (((1-Y_INTERCEPT_HOLD)/TURNING_MAX_HOLD)*degToGo+Y_INTERCEPT_HOLD));
                }
                if(averageDistEncoder > DRIVING_THRESHOLD) {
                    leftMotorOutput = -(.002 * averageDistEncoder + .25);
                    rightMotorOutput = -(.002 * averageDistEncoder + .25);
                } else if(averageDistEncoder < -DRIVING_THRESHOLD){
                    leftMotorOutput = -(.002 * averageDistEncoder - .25);
                    rightMotorOutput = -(.002 * averageDistEncoder - .25);
                } 
                break;
            case State.FUNCT_STANDBY:
                leftMotorOutput = (wantedLeftSpeed * .5) + (leftMotorOutput * .5);
                rightMotorOutput = (wantedRightSpeed * .5) + (leftMotorOutput *.5);
               break;
            default:
                log.logMessage("UNKNOWN STATE: " + autoFunctionState);
        }

        switch(drivesState){
            case State.DRIVES_LOW_GEAR:
                if(((averageSpeed > UP_SHIFT_THRESHOLD && !manualShifting) || (needsToManuallyShiftUp && manualShifting)) && !forceLowGear){
                    needsToManuallyShiftUp = false;
                    needsToManuallyShiftDown = false;
                    log.logMessage("Up Shift!");
                    shifter.set(!LOW_GEAR);
                    drivesState = State.DRIVES_SHIFT_HIGH_GEAR;
                    shiftTime = Timer.getFPGATimestamp();
                }
                break;
            case State.DRIVES_HIGH_GEAR:
                if((averageSpeed < DOWN_SHIFT_THRESHOLD && !manualShifting) || (needsToManuallyShiftDown && manualShifting)){
                    needsToManuallyShiftUp = false;
                    needsToManuallyShiftDown = false;
                    log.logMessage("Down Shift!");
                    shifter.set(LOW_GEAR);
                    drivesState = State.DRIVES_SHIFT_LOW_GEAR;
                    shiftTime = Timer.getFPGATimestamp();
                }
                break;
            case State.DRIVES_SHIFT_LOW_GEAR:
                if(Timer.getFPGATimestamp() > shiftTime + SHIFT_TIME){
                    drivesState = State.DRIVES_LOW_GEAR;
                }
                setSpeed(((wantedLeftSpeed > 0) ? MOTOR_SHIFTING_SPEED : -MOTOR_SHIFTING_SPEED), ((wantedRightSpeed > 0) ? MOTOR_SHIFTING_SPEED : -MOTOR_SHIFTING_SPEED));
                break;
            case State.DRIVES_SHIFT_HIGH_GEAR:
                if(Timer.getFPGATimestamp() > shiftTime + SHIFT_TIME){
                    drivesState = State.DRIVES_HIGH_GEAR;
                }
                setSpeed(((wantedLeftSpeed > 0) ? MOTOR_SHIFTING_SPEED : -MOTOR_SHIFTING_SPEED), ((wantedRightSpeed > 0) ? MOTOR_SHIFTING_SPEED : -MOTOR_SHIFTING_SPEED));
                break;
            default:
                log.logError("Unknown state for drives: " + drivesState);
                break;                   
        }
        //LEFT MOTORS
        leftFrontDrives.set(leftMotorOutput);
        leftRearDrives.set(leftMotorOutput);

        //RIGHT MOTORS
        rightFrontDrives.set(-rightMotorOutput);
        rightRearDrives.set(-rightMotorOutput); 
        updatedSmartDashboard();
    }
    
    /**
     * Logs info about the drives subsystem
     */
    public void logInfo(){
        log.logMessage("Gyro: " + gyro.getAngle());
        log.logMessage("Gyro Voltage: " + gyroAnalog.getVoltage());
        log.logMessage("Left Wanted: " + wantedLeftSpeed + " Right Wanted: " + wantedRightSpeed);
        log.logMessage("Left Motor Output: " + leftMotorOutput + " Right Motor Output: " + rightMotorOutput);
        log.logMessage("Left Encoder Distance: " + leftEncoderData.getDistance() + " Right Encoder Distance: " + rightEncoderData.getDistance());
        log.logMessage("Left Encoder Rate: " + leftEncoderData.getSpeed() + " Right Encoder Rate:" + rightEncoderData.getSpeed());
        log.logMessage("Shift State = " + State.getState(drivesState) + " Functions State: " + State.getState(autoFunctionState));
        log.logMessage("Average Runtime: " + getAverageRuntime() + "seconds");
        
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
            speed = ((wantedSpeed - currentSpeed) / MAX_ROBOT_SPEED / 3) + currentOutput;
        }

        return speed;
    }
    
    /**
     * Set the speed of the individual drives in inches per second.
     *
     * @param left speed of left drives system in Inches per Second if in auto,
     * else motor output
     * @param right speed of left drives system in Inches per Second if in auto,
     * else motor output
     */
    public void setSpeed(double left, double right){
        wantedLeftSpeed = left;
        wantedRightSpeed = right;
    }
    
    /**
     * turn the robot
     * @param degrees - the desired number of degrees to turn. Use negative 
     * values to turn left 
     */
    public void turn(double degrees){
        resetGyro();
        desiredAngle = degrees;
        autoFunctionState = State.FUNCT_TURNING;
    }
    
    /**
     * Drives straight for inches number of inches. Self correcting
     *
     * @param inches the desired number of inches to drive. Use negatives to go
     * backwards
     * @param speed the speed in Inches per Second you want to go
     */
    public void driveStraight(double inches, double speed){
        resetEncoders();
        inchesToGo = inches;
        setSpeed(speed, speed);
        autoFunctionState = State.FUNCT_DRIVE_STRAIGHT;
    }
    /**
     * Forces drives to stay in low gear or to release it from low gear
     * @param stayInLowGear - whether or not to force low gear
     */
    public void forceLowGear(boolean stayInLowGear){
        forceLowGear = stayInLowGear;
    }
     /**
     * Sets the drives to shift up if manualShifting is true
     */
    public void manualShiftUp(){
        needsToManuallyShiftUp = true;
        log.logMessage("Manually shifting Up");
    }
    /**
     * Sets the drives to shift down if manualShifting is true
     */
    public void manualShiftDown(){
        needsToManuallyShiftDown = true;
        log.logMessage("Manually shifting Down");
    }
    /**
     * Sets manualShifting to manual. If manualShifting is true, it will disable
     * autoshifting and rely only on manual shifting
     * @param manual - whether or not manual shifting is enabled
     */
    public void setManualShifting(boolean manual){
        manualShifting = manual;
    }
    /**
     * Resets the encoders and gyro and sets the state to HOLD_POS. It will
     * prioritize turning over driving. It will maintain this position until
     * stopHoldPos() is called
     */
    public void startHoldPos(){
        if(autoFunctionState != State.FUNCT_HOLD_POS){
            resetSensors();
            autoFunctionState = State.FUNCT_HOLD_POS;
        }
    }
    /**
     * Stops the holding of the position saved when startHoldPos() was called
     */
    public void stopHoldPos(){
        autoFunctionState = State.FUNCT_STANDBY;
    }
    
    /**
     * Resets all the the encoders
     */
    private void resetEncoders(){
        rightDrivesEncoder.reset();
        rightEncoderData.reset();
        leftDrivesEncoder.reset();
        leftEncoderData.reset();
    }
    
    /**
     * Resets the gyro
     */
    private void resetGyro(){
        gyro.reset();
    }
    
    /**
     * Resets the gyro and encoders
     */
    private void resetSensors(){
        resetEncoders();
        resetGyro();
    }
    
    private boolean gyroCheck(){
        if(gyroAnalog.getVoltage() > GYRO_ERROR){
           return true;
        }else{
            log.logMessage("NO GYRO STUPID");
            return false;
        }
    }
    
    /**
     * Returns if the last command is done
     */
    public boolean isLastCommandDone() {
        return (autoFunctionState == State.FUNCT_HOLD_POS || autoFunctionState == State.FUNCT_STANDBY);
    }
    
    private void updatedSmartDashboard(){
        SmartDashboard.putBoolean(smartAutoShiftingName, !manualShifting);
    }

    public void liveWindow() {
        LiveWindow.addActuator(subsystemName, "Right Front", rightFrontDrives);
        LiveWindow.addActuator(subsystemName, "Right Back", rightFrontDrives);
        LiveWindow.addSensor(subsystemName, "Right Encoder", rightDrivesEncoder);
        LiveWindow.addActuator(subsystemName, "Left Front", rightFrontDrives);
        LiveWindow.addActuator(subsystemName, "Left Back", rightFrontDrives);
        LiveWindow.addSensor(subsystemName, "Left Encoder", leftDrivesEncoder);
        LiveWindow.addActuator(subsystemName, "Shifting", shifter);
        LiveWindow.addSensor(subsystemName, "GYRO", gyro);
        SmartDashboard.putBoolean(smartAutoShiftingName, true);
    }

    public int sleepTime(){
        return 10;
    }
    
    private static class State{
        static final int DRIVES_LOW_GEAR           = 1;
        static final int DRIVES_SHIFT_LOW_GEAR     = 2;
        static final int DRIVES_HIGH_GEAR          = 4;
        static final int DRIVES_SHIFT_HIGH_GEAR    = 5;
        static final int FUNCT_TURNING            = 6;
        static final int FUNCT_DRIVE_STRAIGHT     = 7;
        static final int FUNCT_HOLD_POS           = 8;
        static final int FUNCT_STANDBY            = 9;
        
        public static String getState(int state){
            switch(state){
                case DRIVES_LOW_GEAR:
                    return "Low Gear";
                case DRIVES_SHIFT_LOW_GEAR:
                    return "Shift Low Gear";
                case DRIVES_HIGH_GEAR:
                    return "High Gear";
                case DRIVES_SHIFT_HIGH_GEAR:
                    return "Shift High Gear";
                case FUNCT_TURNING:
                    return "Turning";
                case FUNCT_DRIVE_STRAIGHT:
                    return "Drive Straight";
                case FUNCT_HOLD_POS:
                    return "Holding current Position";
                case FUNCT_STANDBY:
                    return "Standby";
        }
            return "UNKOWN MODE";
        }
    }

}
