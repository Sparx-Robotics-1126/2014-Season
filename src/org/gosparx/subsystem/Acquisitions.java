package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.gosparx.IO;
import org.gosparx.sensors.EncoderData;
import org.gosparx.util.Logger;

/**
 *Tried to improve Acquisitions
 * @author Connor
 */

/*/
1) Rotate from shooting mode to acquire mode
2)Encoder/limit switch tells us we are down
3)extend/turn on motors to aquire
4)Limit Switch/operator override
5)REtract smaller cylinder
6)Retact large cylinder at an angle (can't go out of the 5 foot height)
7)Go to an angle (shooting/safe)
8)Extend small cylinder before firing
9)DO IT ALL AGAIN

STATES:
Rotate Up
Rotate Down
Rotate_Ready_to_Extend
Acquiring - NO BALL
Acquired - HAVE BALL
Ready_to_retract(both small and large)
Ready_to_shoot
Safe_State
/*/

public class Acquisitions extends GenericSubsystem{

    /**
     * The Acquisitions object.
     * Is returned in getInstance
     */
    private static Acquisitions acquisitions;
    
    /*/*************MOTORS/SENSORS/SOLENOIDS******************** /*/
    
    /**
     * Used to control the angle of the shooter
     * mini-CIM driven with a 55/12 reduction (motor/output)
     */
    private CANJaguar rotatingMotor;
    private Jaguar rotatingMotorPWM;
    
    /**
     * Used to control the intake rollers
     * Bag Motor/Fisher Price driven with a 5/1 reduction (motor/output)
     */
    private CANJaguar acqRoller;
    private Jaguar acqRollerPWM;
    
    /**
     * Used to fully extend the acquisition rollers to be able to acquire balls
     */
    private Solenoid acqLongPnu;
    
    /**
     * Used to move acquisition rollers out of the way of the ball when firing.
     * Should be extended once match begins
     */
    private Solenoid acqShortPnu;
    
    /**
     * Limit Switch. Mounted on the acquisition rollers
     * Detects when a ball is between the first and second roller
     */
    private DigitalInput ballDetector;
    
    /**
     * Two Limit Switches run in series. NORMALLY CLOSED
     * Mounted on the pivoting frame. 
     * Detect when the shooter has reached its maximum angle (safe position)
     */
    private DigitalInput upperLimit; 
    
    /**
     * Two Limit Switches run in series. NORMALLY CLOSED
     * Mounted on the drive base.
     * Detect when the shooter has reached it maximum angle (acquiring position)
     */
    private DigitalInput lowerLimit;
            
    /**
     * Is attached to the motor that drives the rotating motion.
     * There is a 55/12 reduction from the encoder to the shooter.
     * Rotating DOWN will give you a negative distance.
     * Rotating UP will gave you a positive distance.
     */
    private Encoder rotateEncoder;
    
    /**
     * Reliable data gotten from the rotateEncoder.
     * Calculates currentSpeed and distance traveled by the encoder.
     */
    private EncoderData rotateEncoderData;
    
    /*/************************FINAL VARIABLES*********************** /*/
    
    /**
     * The extended state of the short cylinder used to move the rollers out of the way of the shooter.
     * Extended = NO interference;
     * Retracted = interference;
     */
    private final static boolean ACQ_SHORT_PNU_EXTENDED = true;//TODO: CHECK
    
    /**
     * The extended state of the long cylinder used to move the rollers into acquiring position.
     * Extended = Able to pick up ball
     * Retracted = In the shooter perimeter (can't pick up balls)
     */
    private final static boolean ACQ_LONG_PNU_EXTENDED = true;//TODO: CHECK
    
    /**
     * The speed at which the rollers pick up a ball.
     * May have to be modified based on design. (slower may be better)
     */
    private final static double INTAKE_ROLLER_SPEED = 1.0;//TODO: CHECK
    
    /**
     * The distance each tick travels. (in degrees)
     */
    private final static double DEGREES_PER_TICK = 0.14064698;//0.007670476; - WORM GEAR
    
    /**
     * Degrees per second
     */
    private final static double ROTATE_UP_SPEED = -80;
    
    /**
     * The angle at which it is legal for the acquisition rollers to extend 
     * without breaking rules
     */
    private final static int ACQ_ROLLER_ALLOWED_TO_EXTEND = 110;//TODO: CHECK
    
    
    /**
     * The angle at which the longer rollers must be retracted to be within the 
     * frame perimeter.
     */ 
    private final static int ACQ_ROLLER_ALLOWED_TO_EXTEND_UPPER = 65;
    
    /**
     * The name of this subsystem.
     * Used in live window to setup all the motors/sensors/solenoid to be grouped with with class
     */
    private final static String subsystemName = "Acquisitions";
    
    /**
     * Close Shooter preset. Use this angle if we are close to the goal.
     */
    private final static int CLOSE_SHOOTER_PRESET = 20;
    
    /**
     * Mid Shooter preset. Use this preset if we are midrange from the goal.
     */
   private final static int MID_SHOOTER_PRESET = 30;
   
   /**
    * Far Shooter preset. Use if we are far from the goal.
    */
   private final static int FAR_SHOOTER_PRESET = 45;
   
   /**
    * The angle where the shooter shifts center of gravity. Used to slow down so
    * we do not slam back into the back supports and limit switches.
    */
   private final static int CENTER_OF_GRAVITY_ANGLE = 25;
   
   /**
    * The angle when we are close to the floor mode, or acquiring. It is used to
    * slow down the motors so we do not bounce back when we attempt to aquire.
    */ 
   private final static int CLOES_TO_ACQUIRING_ANGLE = 90;
   
   /**
    * The time in seconds the limit switch must be pressed for so that it 
    * actually counts. It is used to prevent noise from triggering acquisitions
    */
   private final static double BALL_DETECT_TIME = 0.1;
   
    /*/************************VARIABLES***************************** /*/
    
    /**
     * The current acquisition state that the robot is in.
     * See the State class to find all possible states
     */
    private int acquisitionState;
    
    /**
     * The wanted state of the system
     */
    private int wantedState;
    
    /**
     * The speed at which the shooter will rotate. (in percentage)
     */
    private double rotationSpeed = 0;
    
    /**
     * The wanted angle of the shooter. Between 0 and 120. 
     * 0 being vertical (safe mode).
     * 120 being acquiring (acquiring mode)
     */
    private double wantedShooterAngle = 0;//Default
    
    /**
     * Stores if we have reset our encoder data to 0 while we are setting home
     */
    private boolean isEncoderDataSet = false;//Not true on power up
    
    /**
     * The last FPGA time we detected a ball at. Used to determine false
     * positives on the limit switch
     */
    private double lastBallDetectTime = 0;
    
    /**
     * Stores if we have hit the limit switch for the ball being sucked in by 
     * acquisitions. Used to determine false positives.
     */ 
    private boolean isBallInRollers = false;
    
    /**
     * The angle in degrees if shooter is in the down position.
     */
    private static final double DOWN_POSITION = 120;
    
    /**
     * The angle in degrees if the shooter is full up.
     */
    private static final double UP_POSITION = 0;
    
    /**
     * The wanted acquisitions speed for the acquisitions motor.
     */ 
    private double wantedAcqSpeed = 0;
    
    /**    
     * The group to display the ready to shoot rectangle on the livewindow under.
     */ 
    private static final String READY_TO_SHOOT_DISPLAY = "Ready To Shoot";
    
    /**
     * The group to put the wanted angle meter on the livewindow under.
     */ 
    private static final String WANTED_ANGLE_DISPLAY = "Wanted Angle";
    
    /**
     * The tolerance in degrees for pivoting.
     */
    private static final double PIVOT_THRESHOLD                             = 1;
    
    /**
     * 
     * @returns the only running thread of Acquisitions.
     * This should be used instead of (new Acquisitions2)
     */
    public static Acquisitions getInstance(){
        if(acquisitions == null){
            acquisitions = new Acquisitions();
        }
        return acquisitions;
    }
    
    /**
     * Constructor for Acquisitions
     * Sets the thread priority and sets the name of the subsystem for logging purposes
     */
    private Acquisitions(){
        super(Logger.SUB_ACQUISITIONS, Thread.NORM_PRIORITY);
    }
    
    /**
     * Initiates all of the motors/sensors/solenoids.
     * Sets the short cylinder to its default position.
     */
    public void init() {
        if(!IO.USE_PWM_CABLES){
            try {
                rotatingMotor = new CANJaguar(IO.CAN_ADRESS_PIVOT);
                acqRoller = new CANJaguar(IO.CAN_ADRESS_ACQ);
            } catch (CANTimeoutException ex) {
                log.logError("CANBus Timeout in Acquisitions init()");
                throw new RuntimeException("CAN timeout in acquisitions init");
            }
        }else{
            rotatingMotorPWM = new Jaguar(IO.DEFAULT_SLOT, IO.PWM_PIVOT);
            acqRollerPWM = new Jaguar(IO.DEFAULT_SLOT, IO.PWM_ACQ);
        }
        acqLongPnu = new Solenoid(IO.DEFAULT_SLOT, IO.ACQ_TOGGLE_CHAN);
        ballDetector = new DigitalInput(IO.DEFAULT_SLOT, IO.ACQ_BALL_DETECTOR);
        upperLimit = new DigitalInput(IO.DEFAULT_SLOT, IO.SHOOTER_SAFE_MODE_CHAN);
        rotateEncoder = new Encoder(IO.DEFAULT_SLOT, IO.PIVOT_ENCODER_CHAN_1, IO.DEFAULT_SLOT, IO.PIVOT_ENCODER_CHAN_2);
        rotateEncoder.setDistancePerPulse(DEGREES_PER_TICK);
        rotateEncoderData = new EncoderData(rotateEncoder, DEGREES_PER_TICK);
        rotateEncoderData.reset();
        lowerLimit = new DigitalInput(IO.DEFAULT_SLOT, IO.SHOOTER_ACQ_MODE_CHAN);
        acqShortPnu = new Solenoid(IO.DEFAULT_SLOT, IO.KEEP_IN_FRAME_CHAN);
        acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);//Puts the rollers out of way of the shooter
        acquisitionState = AcqState.ROTATE_UP;//default state
        wantedState = AcqState.SAFE_STATE;
    }

    /**
     * Main run method. Is called through genericSubsystem.
     * @throws Exception - if thrown then the thread will try to restart itself
     */
    public void execute() throws Exception {
        if (ds.isTest() && ds.isDisabled()) {//ALL VALUES NEED TO BE SET TO 0
            if (!IO.USE_PWM_CABLES) {
                rotatingMotor.setX(0);
                acqRoller.setX(0);
            } else {
                rotatingMotorPWM.set(0);
                acqRollerPWM.set(0);
            }
        }
        rotateEncoderData.calculateSpeed();//Calculates the distance and speed of the encoder
        isBallInRollers = !ballDetector.get();
        switch (acquisitionState) {
            case AcqState.ROTATE_UP://rotate shooter up
                if (wantedShooterAngle == UP_POSITION && upperLimit.get()) {//straight up and down
                    rotationSpeed = 0;
                    isEncoderDataSet = true;
                    rotateEncoderData.reset();
                    acquisitionState = AcqState.SAFE_STATE;
                } else if (Math.abs(wantedShooterAngle - PIVOT_THRESHOLD) >= rotateEncoderData.getDistance() && isEncoderDataSet) {
                    rotationSpeed = 0;
                    acquisitionState = wantedState;
                } else {
                    if (rotateEncoderData.getDistance() > CENTER_OF_GRAVITY_ANGLE) {
                        if (rotateEncoderData.getSpeed() < ROTATE_UP_SPEED) {
                            rotationSpeed += -.05;
                        } else {
                            rotationSpeed -= -.05;
                        }
                    } else {
                        rotationSpeed = 0.3;
                    }
                }
                if (ACQ_ROLLER_ALLOWED_TO_EXTEND >= rotateEncoderData.getDistance()
                        && acqShortPnu.get() == ACQ_SHORT_PNU_EXTENDED && wantedState != AcqState.READY_TO_SHOOT) {
                    acquisitionState = AcqState.ROTATE_READY_RETRACT;
                }

                if (ACQ_ROLLER_ALLOWED_TO_EXTEND_UPPER >= rotateEncoderData.getDistance()
                        && acqLongPnu.get() == ACQ_LONG_PNU_EXTENDED) {
                    acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                }
                wantedAcqSpeed = 0;//turns motors off
                break;
            case AcqState.ROTATE_DOWN://rotate shooter down
                if (wantedShooterAngle == DOWN_POSITION && lowerLimit.get()) {//limit Switch reads false when touched
                    rotationSpeed = 0;
                    acquisitionState = wantedState;
                } else if (Math.abs(wantedShooterAngle + PIVOT_THRESHOLD) <= rotateEncoderData.getDistance()) {
                    rotationSpeed = 0;
                    acquisitionState = wantedState;
                } else {
                    if (rotateEncoderData.getDistance() < CLOES_TO_ACQUIRING_ANGLE) {
                        rotationSpeed = -0.9;//MAY WANT TO RAMP
                    } else {
                        rotationSpeed = -0.45;
                    }
                }
                if (ACQ_ROLLER_ALLOWED_TO_EXTEND <= rotateEncoderData.getDistance()
                        && acqLongPnu.get() != ACQ_LONG_PNU_EXTENDED) {
                    acquisitionState = AcqState.ROTATE_READY_TO_EXTEND;
                }
                wantedAcqSpeed = 0;//turns motors off
                break;
            case AcqState.ROTATE_READY_TO_EXTEND://angle at which it is safe to extend the rollers
                acqLongPnu.set(ACQ_LONG_PNU_EXTENDED);
                acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                acquisitionState = AcqState.ROTATE_DOWN;
                break;
            case AcqState.ROTATE_READY_RETRACT://angle at which it is safe to extend the rollers
                wantedAcqSpeed = 0;
                acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                acqShortPnu.set(!ACQ_SHORT_PNU_EXTENDED);
                acquisitionState = AcqState.ROTATE_UP;
                break;
            case AcqState.ACQUIRING://Rollers are running and we are getting a ball
                acqLongPnu.set(ACQ_LONG_PNU_EXTENDED);
                acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                wantedAcqSpeed = INTAKE_ROLLER_SPEED;//Turns rollers on
                if (isBallInRollers && Timer.getFPGATimestamp() - lastBallDetectTime >= BALL_DETECT_TIME) {
                    acquisitionState = AcqState.ACQUIRED;
                    log.logMessage("Ball Detected!");
                } else if (isBallInRollers && lastBallDetectTime == 0) {
                    lastBallDetectTime = Timer.getFPGATimestamp();
                } else if (!isBallInRollers) {
                    lastBallDetectTime = 0;
                }
                break;
            case AcqState.ACQUIRED://limit switch has been pressed - short cylinder retracts
                wantedAcqSpeed = 0;
                acqShortPnu.set(!ACQ_SHORT_PNU_EXTENDED);//Ball can't escape
                break;
            case AcqState.EJECT_BALL://ball is being ejected from robot through rollers
                acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);//Ball rolls right on out
                break;
            case AcqState.READY_TO_RETRACT://The maximum angle to be at before an over 5' penalty
                acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                break;
            case AcqState.READY_TO_SHOOT://Rollers are out of the way, Shooting angle is set
                acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                break;
            case AcqState.SAFE_STATE://Shooter is in the robots perimeter
                break;
            case AcqState.OFF_STATE://Something has gone wrong. All motors are set to 0.0
                rotationSpeed = 0;
                acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                wantedAcqSpeed = 0;
                break;
        }
        setPivotMotor(rotationSpeed);
        setAcquiringMotor(wantedAcqSpeed);
        updateSmartDashboard();
    }
    
    /**
     * Set the acquiring motors to value
     * @param value - the desired motor output
     */ 
    private void setAcquiringMotor(double value){
        if(!IO.USE_PWM_CABLES){
            try {
                acqRoller.setX(value * -1);//motor runs backwards. (silly motors)
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }else{
            acqRollerPWM.set(value * -1);
        }
    }
    
    /**
     * If the current state is in ready to shoot then it is ok to shoot
     * @return if the acquisitions system is ready to shoot or not 
     */
    public boolean readyToShoot(){
        return(acquisitionState == AcqState.READY_TO_SHOOT);
    }
    
    /**
     * Logs relevant info
     */ 
    public void logInfo(){
        rotateEncoderData.calculateSpeed();
       log.logMessage("State: " + AcqState.getStateName(acquisitionState));
       log.logMessage("Wanted State: " + AcqState.getStateName(wantedState));
       log.logMessage("Wanted Angle: " + wantedShooterAngle);
       log.logMessage("Rotate Encoder: " + rotateEncoderData.getDistance());
       log.logMessage("Rotate Motor: " + rotationSpeed);
    }
    
    /**
     * The state you want the system to enter
     * @param state - get from AcqsitionState 
     */
    public void setMode(int state){
            wantedState = state;
            if(isEncoderDataSet){
                switch(state){
                    case AcqState.ACQUIRING:
                        wantedShooterAngle = DOWN_POSITION;
                        acquisitionState = AcqState.ROTATE_DOWN;
                        break;
                    case AcqState.SAFE_STATE:
                        wantedShooterAngle = UP_POSITION;
                        acquisitionState = AcqState.ROTATE_UP;
                        break;
                    case AcqState.READY_TO_SHOOT:
                        setPreset(AcqState.MIDDLE_SHOOTER_PRESET);
                        break;
                    case AcqState.EJECT_BALL:
                        wantedShooterAngle = DOWN_POSITION;//Acquiring
                        acquisitionState = AcqState.ROTATE_DOWN;
                        break;
                    case AcqState.OFF_STATE:
                        acquisitionState = AcqState.OFF_STATE;
                }
            }
    }

    /**
     * Sets the wanted angle of the shooter.
     * 0 - straight up and down
     * 120 - acquiring
     * @param preset 
     */
    public void setPreset(int preset){
        if(ds.isAutonomous()){
            wantedState = AcqState.READY_TO_SHOOT;
        }
        switch(preset){
            case AcqState.CLOSE_SHOOTER_PRESET:
                setAngle(CLOSE_SHOOTER_PRESET);
                break;
            case AcqState.MIDDLE_SHOOTER_PRESET:
                setAngle(MID_SHOOTER_PRESET);
                break;
            case AcqState.FAR_SHOOTER_PRESET:
                setAngle(FAR_SHOOTER_PRESET);
                break;
            default:
                wantedShooterAngle = UP_POSITION;
        }
    }
    
    /**
     * Sets the wanted shooting angle of the robot. And rotates the shooter
     * @param angle - wanted angle in degrees.
     * UP_POSITION - straight up
     * DOWN_POSITION - acquiring
     */
    private void setAngle(int angle){
        wantedShooterAngle = angle;
        rotateEncoderData.calculateSpeed();
        if(rotateEncoderData.getDistance() > wantedShooterAngle){
            acquisitionState = AcqState.ROTATE_UP;
        }else{
            acquisitionState = AcqState.ROTATE_DOWN;
        }
    }
    
    /**
     * Add an offset to the current position of the shooter.
     * @param offset - the offset in degrees to rotate the shooter. Negative 
     *                 goes up, positive goes down.
     */ 
    public void addOffset(int offset){
        wantedShooterAngle += offset;
        if(wantedShooterAngle < UP_POSITION){
            wantedShooterAngle = UP_POSITION;
        }else if(wantedShooterAngle > DOWN_POSITION){
            wantedShooterAngle = DOWN_POSITION;
        }
        if(offset < 0){
            acquisitionState = AcqState.ROTATE_UP;
        }else{
            acquisitionState = AcqState.ROTATE_DOWN;
        }
        wantedState = AcqState.READY_TO_SHOOT;
    }
    
    /**
     * @return if the encoder data has been set.
     */ 
    public boolean isAcquisitionsReady(){
        return isEncoderDataSet;
    }
    
    /**
     * Sets the pivot motor output.
     * @param speed - the desired output of the pivot motor.
     */ 
    private void setPivotMotor(double speed){
        if(!IO.USE_PWM_CABLES){
            try {
                rotatingMotor.setX(speed);
            } catch (CANTimeoutException ex) {
                log.logError("CAN Timeout while setting pivot motor: " + ex.getMessage());
            }
        }else{
            rotatingMotorPWM.set(speed);
        }
    }
    
    /**
     * Tells auto if the next command can start
     * @param doneState
     * @return true if the command is done or false if not
     */
    public boolean isLastCommandDone(int doneState){
        return (doneState == acquisitionState);
    }
    
    /**
     * Update the info on the smart dashboard.
     */ 
    private void updateSmartDashboard(){
        SmartDashboard.putBoolean(READY_TO_SHOOT_DISPLAY, readyToShoot());
        SmartDashboard.putNumber(WANTED_ANGLE_DISPLAY, wantedShooterAngle);
    }

    
    /**
     * Is called right after init().
     * This groups all of the motors/sensors/solenoid together.
     * Sets up the live window screen used in test mode to control each system manually.
     */
    public void liveWindow() {
        if(!IO.USE_PWM_CABLES){
            LiveWindow.addActuator(subsystemName, "Pivot", rotatingMotor);
            LiveWindow.addActuator(subsystemName, "Acquisitions", acqRoller);
        }else{
            LiveWindow.addActuator(subsystemName, "Pivot", rotatingMotorPWM);
            LiveWindow.addActuator(subsystemName, "Acquisitions", acqRollerPWM);
        }
        LiveWindow.addActuator(subsystemName, "Small Cylinder", acqShortPnu);
        LiveWindow.addActuator(subsystemName, "Large Cylinder", acqLongPnu);
        LiveWindow.addSensor(subsystemName, "Upper Limit Switch", upperLimit);
        LiveWindow.addSensor(subsystemName, "Lower Limit Switch", lowerLimit);
        LiveWindow.addSensor(subsystemName, "Ball Detector", ballDetector);
        SmartDashboard.putBoolean(READY_TO_SHOOT_DISPLAY, false);
         SmartDashboard.putNumber(WANTED_ANGLE_DISPLAY, 0);
    }

    public int sleepTime() {
        return 20;
    }
    
    /**
     * Contains all the possible acquisition states.
     * STATES:
     * ROTATE_UP                - rotate shooter up
     * ROTATE_DOWN              - rotate shooter down
     * ROTATE_READY_TO_EXTEND   - angle at which it is safe to extend the rollers
     * ACQUIRING                - Rollers are running and we are getting a ball
     * ACQUIRED                 - limit switch has been pressed - short cylinder retracts
     * EJECT_BALLS              - ball is being ejected from robot through rollers
     * READY_TO_RETRACT         - The maximum angle to be at before an over 5' penalty
     * READY_TO_SHOOT           - Rollers are out of the way, Shooting angle is set
     * SAFE_STATE               - Shooter is in the robots perimeter
     * OFF_STATE                - Something has gone wrong. All motors are set to 0.0
     */
    public static class AcqState{
        public static final int ROTATE_UP = 1;
        public static final int ROTATE_DOWN = 2;
        public static final int ROTATE_READY_TO_EXTEND = 3;
        public static final int ACQUIRING = 4;
        public static final int ACQUIRED = 5;
        public static final int EJECT_BALL = 6;
        public static final int READY_TO_RETRACT = 7;
        public static final int ROTATE_READY_RETRACT = 8;
        public static final int READY_TO_SHOOT = 9;
        public static final int SAFE_STATE = 10;
        public static final int OFF_STATE = 11;
        //USED FOR PRESETS:
        public static final int CLOSE_SHOOTER_PRESET = 20;
        public static final int MIDDLE_SHOOTER_PRESET = 21;
        public static final int FAR_SHOOTER_PRESET = 22;
        
        /**
         * @param state - the state to get the string version of
         * @return A string version of the state.
         */
        public static String getStateName(int state){
            switch(state){
                case ROTATE_UP:
                    return "Rotating Up";
                case ROTATE_DOWN:
                    return "Rotating Down";
                case ROTATE_READY_TO_EXTEND:
                    return "Rotate Ready to Extend";
                case ROTATE_READY_RETRACT:
                    return "Rotate Ready tp Retract";
                case ACQUIRING:
                    return "Acquring";
                case ACQUIRED:
                    return "Acquired";
                case EJECT_BALL:
                    return "Reject ball";
                case READY_TO_RETRACT:
                    return "Ready to Retract";
                case READY_TO_SHOOT:
                    return "Ready to Shoot";
                case SAFE_STATE:
                    return "Safe State";
                case OFF_STATE:
                    return "Off State";
                default:
                    return "UNKNOWN";
            }
        }
    }
    
}
