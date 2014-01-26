package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import org.gosparx.IO;
import org.gosparx.sensors.EncoderData;

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
     * The distance the robot travels per tick of the encoder.
     */
    private static final double DIST_PER_TICK           = 0.047;
    
    /**
     * The absolute value of the speed at which the motors must be going to 
     * shift.
     */
    private static final double MOTOR_SHIFTING_SPEED    = 40;
    
    /**
     * The position of the solenoid when in low gear.
     */
    private static final boolean LOW_GEAR               = false;
    
    /**
     * The position of the solenoid when in high gear.
     */
    private static final boolean HIGH_GEAR              = !LOW_GEAR;
    
    /**
     * The speed (inches per second) that we shift up into high gear at.
     */
    private static final double UP_SHIFT_THRESHOLD      = 42;    
    /**
     * The speed (inches per second) that we shift down into low gear at.
     */
    private static final double DOWN_SHIFT_THRESHOLD    = 15;
    
    /**
     * The time (seconds) we wait for the robot to shift before resuming driver 
     * control.
     */
    private static final double SHIFT_TIME              = .005;
    
    /**
     * The max speed (inches per second) that the robot can obtain.
     */
    public static final double MAX_ROBOT_SPEED         = 168;
    
    /**
     * The accuracy in degrees for turning 
     */
    private static final double TURNING_THRESHOLD                        =  1.5;
    
    /**
     * The accuracy in inches for turning
     */
    private static final double DRIVING_THRESHOLD                         = .75;
    
    /**
     * Time in seconds between logging the desired speed 
     */
    private final double LOG_EVERY = 5.0;
    
    /**
     * The accuracy in inches for turning
     */
    private static final double DRIVING_THRESHOLD                         = .75;
    
    /**
     * Time in seconds between logging the desired speed 
     */
    private final double LOG_EVERY = 5.0;
    
    /**
     * This is the speed in inches per second we want the left side of the 
     * drives to achieve.
     */
    private double wantedLeftSpeed;
    
    /**
     * Average Encoder distance for both the left and right encoder.
     */
    private double averageEncoderDiastance;
    
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
    public Encoder leftDrivesEncoder;
    
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
     * The last time we logged a message to the logger.
     */
    private double lastLogTime = 0;
    
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
     * Stores if the robot is currently turning.
     */ 
    private boolean isTurning;
    
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
     * The current state of the drives
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
    private double averageDistEncoder                                   = 0.0;
        
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
    private double averageDistEncoder                                   = 0.0;
        
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
 
        gyro = new Gyro(IO.GYRO_ANALOG);
        gyro.setPIDSourceParameter(PIDSource.PIDSourceParameter.kAngle);
        gyro.setSensitivity(.0067);
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
        drivesState = State.LOW_GEAR;
        resetSensors();
        while(true){
            currentAngle = gyro.getAngle();
            leftEncoderData.calculateSpeed();
            rightEncoderData.calculateSpeed();
            leftCurrentSpeed = leftEncoderData.getSpeed();
            rightCurrentSpeed = rightEncoderData.getSpeed();
            
            leftMotorOutput = getMotorOutput(wantedLeftSpeed, leftCurrentSpeed, leftMotorOutput);
            rightMotorOutput = getMotorOutput(wantedRightSpeed, rightCurrentSpeed, rightMotorOutput);
            
            double averageSpeed = Math.abs((leftCurrentSpeed+rightCurrentSpeed)/2);
            
            averageEncoderDiastance = (leftDrivesEncoder.getDistance() + rightDrivesEncoder.getDistance())/2;
            switch(drivesState){
                case State.LOW_GEAR:
                    if(((averageSpeed > UP_SHIFT_THRESHOLD && !manualShifting) || (needsToManuallyShiftUp && manualShifting)) && !forceLowGear){
                        needsToManuallyShiftUp = false;
                        needsToManuallyShiftDown = false;
                        log.logMessage("Up Shift!");
                        shifter.set(HIGH_GEAR);
                        drivesState = State.SHIFT_HIGH_GEAR;
                        shiftTime = Timer.getFPGATimestamp();
                    }
                    break;
                case State.HIGH_GEAR:
                    if((averageSpeed < DOWN_SHIFT_THRESHOLD && !manualShifting) || (needsToManuallyShiftDown && manualShifting)){
                        needsToManuallyShiftUp = false;
                        needsToManuallyShiftDown = false;
                        log.logMessage("Down Shift!");
                        shifter.set(LOW_GEAR);
                        drivesState = State.SHIFT_LOW_GEAR;
                        shiftTime = Timer.getFPGATimestamp();
                    }
                    break;
                case State.SHIFT_LOW_GEAR:
                    if(Timer.getFPGATimestamp() > shiftTime + SHIFT_TIME){
                        drivesState = State.LOW_GEAR;
                    }
                    setSpeed(MOTOR_SHIFTING_SPEED, MOTOR_SHIFTING_SPEED);
                    break;
                case State.SHIFT_HIGH_GEAR:
                    if(Timer.getFPGATimestamp() > shiftTime + SHIFT_TIME){
                        drivesState = State.HIGH_GEAR;
                    }
                    setSpeed(MOTOR_SHIFTING_SPEED, MOTOR_SHIFTING_SPEED);
                    break;
                case State.TURNING:
                    if(desiredAngle - currentAngle > 0){
                        leftMotorOutput = (.0044 * (desiredAngle - currentAngle)) + .20;
                        rightMotorOutput = -((.0044 * (desiredAngle - currentAngle)) + .20);
                    } else if(desiredAngle - currentAngle < 0){
                        leftMotorOutput = (.0044 * (desiredAngle - currentAngle)) - .20;
                        rightMotorOutput = -((.0044 * (desiredAngle - currentAngle)) - .20);
                    }
                    log.logMessage("Left Speed: " + leftMotorOutput + " Right Speed: " + rightMotorOutput);
                    if (Math.abs(desiredAngle - currentAngle) < TURNING_THRESHOLD) {
                        log.logMessage("Done Turning");
                        isTurning = false;
                        leftMotorOutput = 0;
                        rightMotorOutput = 0;
                        startHoldPos();
                    }
                    if(DriverStation.getInstance().isOperatorControl() || DriverStation.getInstance().isTest()){
                        drivesState = State.LOW_GEAR;
                    }
                    break;
                case State.DRIVE_STRAIGHT:
                    if(inchesToGo - leftDrivesEncoder.getDistance() > 0) {
//                        leftMotorOutput = getMotorOutput(4*(inchesToGo-leftDrivesEncoder.getDistance()), leftDrivesEncoder.getRate(), leftMotorOutput);
                        setSpeed(30, 30);
                    }
                    if(inchesToGo - rightDrivesEncoder.getDistance() > 0 ){
//                        rightMotorOutput = getMotorOutput(4*(inchesToGo-rightDrivesEncoder.getDistance()), rightDrivesEncoder.getRate(), rightMotorOutput);
//                        rightMotorOutput = 0.5;
                    }
                    if(leftDrivesEncoder.getDistance() - rightDrivesEncoder.getDistance() > 1){
//                        leftMotorOutput = leftMotorOutput * 0.75;
//                        log.logMessage("Slowing Down Left Motor");
                    }else if(rightDrivesEncoder.getDistance() - leftDrivesEncoder.getDistance() > 1){
//                        rightMotorOutput = rightMotorOutput * 0.75;
//                        log.logMessage("Slowing Down Right Motor");
                    }
                    if(Math.abs(leftDrivesEncoder.getDistance() - inchesToGo) < DRIVING_THRESHOLD){
                        leftMotorOutput = 0;
                    }
                    if(Math.abs(rightDrivesEncoder.getDistance() - inchesToGo) < DRIVING_THRESHOLD){
                        rightMotorOutput = 0;
                    }
                    if((Math.abs(rightDrivesEncoder.getDistance() - inchesToGo) < DRIVING_THRESHOLD) && (Math.abs(leftDrivesEncoder.getDistance() - inchesToGo) < DRIVING_THRESHOLD)){
                        log.logMessage("Done Driving Strait.");
                        logDrivesInfo();
                        resetSensors();
                        drivesState = State.HOLD_POS;
                    }
                    if(DriverStation.getInstance().isOperatorControl() || DriverStation.getInstance().isTest()){
                        drivesState = State.LOW_GEAR;
                    }
                    break;
                case State.HOLD_POS:
                    averageDistEncoder = (leftDrivesEncoder.getDistance() + rightDrivesEncoder.getDistance())/2;
                    if(gyro.getAngle() > TURNING_THRESHOLD){
                        leftMotorOutput = -((.0048 * (gyro.getAngle())) + .15);
                        rightMotorOutput = ((.0048 * (gyro.getAngle())) + .15);
                    } else if(gyro.getAngle() < -TURNING_THRESHOLD){
                        leftMotorOutput = -((.0048 * (gyro.getAngle())) - .15);
                        rightMotorOutput = ((.0048 * (gyro.getAngle())) - .15);
                    } else if(averageDistEncoder > DRIVING_THRESHOLD) {
                        leftMotorOutput = -(.002 * averageDistEncoder + .25);
                        rightMotorOutput = -(.002 * averageDistEncoder + .25);
                    } else if(averageDistEncoder < -DRIVING_THRESHOLD){
                        leftMotorOutput = -(.002 * averageDistEncoder - .25);
                        rightMotorOutput = -(.002 * averageDistEncoder - .25);
                    } else{
                        leftMotorOutput = 0;
                        rightMotorOutput = 0;
                    }
                    break;
                default:
                    log.logError("Unknown state for drives: " + drivesState);
            }
            leftFrontDrives.set(leftMotorOutput);
            leftRearDrives.set(leftMotorOutput);
            rightFrontDrives.set(-rightMotorOutput);
            rightRearDrives.set(-rightMotorOutput);
            
            if(Timer.getFPGATimestamp() - LOG_EVERY >= lastLogTime && ds.isEnabled()){
                lastLogTime = Timer.getFPGATimestamp();
                logDrivesInfo();
            }
            Thread.sleep(10);
        }
    }
    
    private void logDrivesInfo(){
        log.logMessage("Left: " + wantedLeftSpeed + " Right: " + wantedRightSpeed);
        log.logMessage("Left Encoder Distance: " + leftEncoderData.getDistance() + " Right Encoder Distance: " + rightEncoderData.getDistance());
        log.logMessage("Left Encoder Rate: " + leftEncoderData.getSpeed() + " Right Encoder Rate:" + rightEncoderData.getSpeed());
        log.logMessage("Drive State = " + State.getState(drivesState));
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
            speed = ((wantedSpeed - currentSpeed) / MAX_ROBOT_SPEED / 2) + currentOutput;
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
     * turn the robot
     * @param degrees - the desired number of degrees to turn. Use negative 
     * values to turn left 
     */
    public void turn(double degrees){
        gyro.reset();
        desiredAngle = degrees;
        isTurning = true;
        drivesState = State.TURNING;
    }
    
    /**
     * Drives straight for inches number of inches. Self correcting
     * @param inches - the desired number of inches to drive. Use negatives to
     *                 go backwards
     */
    public void driveStraight(double inches){
        drivesState = State.DRIVE_STRAIGHT;
        inchesToGo = inches;
        leftDrivesEncoder.reset();
        rightDrivesEncoder.reset();
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
        System.out.println("Manually shifting Up");
    }
    /**
     * Sets the drives to shift down if manualShifting is true
     */
    public void manualShiftDown(){
        needsToManuallyShiftDown = true;
        System.out.println("Manually shifting Down");
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
        resetSensors();
        drivesState = State.HOLD_POS;
    }
    /**
     * Stops the holding of the position saved when startHoldPos() was called
     */
    public void stopHoldPos(){
        drivesState = State.LOW_GEAR;
    }
    
    private void resetEncoders(){
        rightDrivesEncoder.reset();
        rightEncoderData.reset();
        leftDrivesEncoder.reset();
        leftEncoderData.reset();
    }
    
    private void resetGyro(){
        gyro.reset();
    }
    
    private void resetSensors(){
        resetEncoders();
        resetGyro();
    }
    
    public boolean isLastCommandDone() {
        return drivesState == State.HOLD_POS;
    }
    
    private static class State{
        static final int LOW_GEAR           = 1;
        static final int SHIFT_LOW_GEAR     = 2;
        static final int HIGH_GEAR          = 4;
        static final int SHIFT_HIGH_GEAR    = 5;
        static final int TURNING            = 6;
        static final int DRIVE_STRAIGHT     = 7;
        static final int HOLD_POS           = 8;
        
        public static String getState(int state){
            switch(state){
                case LOW_GEAR:
                    return "Low Gear";
                case SHIFT_LOW_GEAR:
                    return "Shift Low Gear";
                case HIGH_GEAR:
                    return "High Gear";
                case SHIFT_HIGH_GEAR:
                    return "Shift High Gear";
                case TURNING:
                    return "Turning";
                case DRIVE_STRAIGHT:
                    return "Drive Straight";
                case HOLD_POS:
                    return "Holding current Position";
        }
            return "UNKOWN MODE";
    private void resetGyro(){
        gyro.reset();
    }
    
    private void resetSensors(){
        resetEncoders();
        resetGyro();
    }
    
    public boolean isLastCommandDone() {
        return drivesState == State.HOLD_POS;
    }
    
    private static class State{
        static final int LOW_GEAR           = 1;
        static final int SHIFT_LOW_GEAR     = 2;
        static final int HIGH_GEAR          = 4;
        static final int SHIFT_HIGH_GEAR    = 5;
        static final int TURNING            = 6;
        static final int DRIVE_STRAIGHT     = 7;
        static final int HOLD_POS           = 8;
        
        public static String getState(int state){
            switch(state){
                case LOW_GEAR:
                    return "Low Gear";
                case SHIFT_LOW_GEAR:
                    return "Shift Low Gear";
                case HIGH_GEAR:
                    return "High Gear";
                case SHIFT_HIGH_GEAR:
                    return "Shift High Gear";
                case TURNING:
                    return "Turning";
                case DRIVE_STRAIGHT:
                    return "Drive Straight";
                case HOLD_POS:
                    return "Holding current Position";
        }
            return "UNKOWN MODE";
        }
    }
}
