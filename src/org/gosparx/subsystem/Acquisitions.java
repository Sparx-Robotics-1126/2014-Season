package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.gosparx.Autonomous;
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
    private final static double DEGREES_PER_TICK = 0.007670476;
    
    private final static double ROTATE_UP_SPEED = -60;
    
    /**
     * The angle at which it is legal for the acquisition rollers to extend without breaking rules
     */
    private final static int ACQ_ROLLER_ALLOWED_TO_EXTEND = 110;//TODO: CHECK
    
    private final static int ACQ_ROLLER_ALLOWED_TO_EXTEND_UPPER = 90;
    
    /**
     * The name of this subsystem.
     * Used in live window to setup all the motors/sensors/solenoid to be grouped with with class
     */
    private final static String subsystemName = "Acquisitions";
    
    /**
     * Close Shooter preset
     */
    private final static int CLOSE_SHOOTER_PRESET = 00;
    
    /**
     * Mid Shooter preset
     */
   private final static int MID_SHOOTER_PRESET = 00;
   
   /**
    * Far Shooter preset
    */
   private final static int FAR_SHOOTER_PRESET = 00;
   
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
    private int wantedShooterAngle = 0;//Default
    
    /**
     * The current state of the bigger general state. Used in findNextCase()
     * Automagicly increments.
     */ 
    private int currentStatePosition = 0;
    
    private boolean isEncoderDataSet = false;//Not true on power up
    
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
            }
        }else{
            rotatingMotorPWM = new Jaguar(IO.DEFAULT_SLOT, IO.PWM_PIVOT);
            acqRollerPWM = new Jaguar(IO.DEFAULT_SLOT, IO.PWM_ACQ);
        }
        acqLongPnu = new Solenoid(IO.ACQ_TOGGLE_CHAN);
        ballDetector = new DigitalInput(IO.ACQ_SWITCH_CHAN);
        upperLimit = new DigitalInput(IO.SHOOTER_SAFE_MODE_CHAN);
        rotateEncoder = new Encoder(IO.DEFAULT_SLOT, IO.PIVOT_ENCODER_CHAN_1, IO.DEFAULT_SLOT, IO.PIVOT_ENCODER_CHAN_2);
        rotateEncoder.setDistancePerPulse(DEGREES_PER_TICK);
        rotateEncoderData = new EncoderData(rotateEncoder, DEGREES_PER_TICK);
        lowerLimit = new DigitalInput(IO.SHOOTER_ACQ_MODE_CHAN);
        acqShortPnu = new Solenoid(IO.KEEP_IN_FRAME_CHAN);
        acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);//Puts the rollers out of way of the shooter
        acquisitionState = AcqState.ROTATE_UP;//default state
        wantedState = AcqState.ROTATE_UP;
    }

    /**
     * Main run method. Is called through genericSubsystem.
     * @throws Exception - if thrown then the thread will try to restart itself
     */
    public void execute() throws Exception {
        rotateEncoderData.reset();
        while(!ds.isTest()){//motors can't be given values during test mode, OR IT DOSEN'T WORK
            rotateEncoderData.calculateSpeed();//Calculates the distance and speed of the encoder
            switch(acquisitionState){                
                case AcqState.ROTATE_UP://rotate shooter up
                    if(wantedShooterAngle == 0 && upperLimit.get()){//straight up and down
                        rotationSpeed = 0;
                        isEncoderDataSet = true;
                        rotateEncoderData.reset();
                        acquisitionState = AcqState.SAFE_STATE;
                    }else if(wantedShooterAngle >= rotateEncoderData.getDistance() && isEncoderDataSet){
                        rotationSpeed = 0;
                        acquisitionState = wantedState;
                    }else{
                        if(rotateEncoderData.getSpeed() < ROTATE_UP_SPEED){
                            rotationSpeed += -.05;
                        }else {
                            rotationSpeed -= -.05; 
                        }
                    }
                    if(ACQ_ROLLER_ALLOWED_TO_EXTEND >= rotateEncoderData.getDistance() 
                            && acqShortPnu.get() == ACQ_SHORT_PNU_EXTENDED)
                        acquisitionState = AcqState.ROTATE_READY_RETRACT;
                    
                    if(ACQ_ROLLER_ALLOWED_TO_EXTEND_UPPER >= rotateEncoderData.getDistance() 
                            && acqLongPnu.get() == ACQ_LONG_PNU_EXTENDED)
                        acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                    break;
                case AcqState.ROTATE_DOWN://rotate shooter down
                    if(wantedShooterAngle == 120 && lowerLimit.get()){//limit Switch reads false when touched
                        rotationSpeed = 0;
                        acquisitionState = wantedState;
                    }else if(wantedShooterAngle <= rotateEncoderData.getDistance()){
                        rotationSpeed = 0;
                        acquisitionState = wantedState;
                    }else{
                        rotationSpeed = -0.5;//MAY WANT TO RAMP
                    }
                    if(lowerLimit.get()){
                        rotationSpeed = 0;
                        log.logMessage("Lower Limit has been tripped, unknown position");
                    }
                    if(ACQ_ROLLER_ALLOWED_TO_EXTEND <= rotateEncoderData.getDistance() 
                            && acqLongPnu.get() != ACQ_LONG_PNU_EXTENDED){
                        acquisitionState = AcqState.ROTATE_READY_TO_EXTEND;
                    }
                    break;
                case AcqState.ROTATE_READY_TO_EXTEND://angle at which it is safe to extend the rollers
                        acqLongPnu.set(ACQ_LONG_PNU_EXTENDED);
                        acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                        acquisitionState = AcqState.ROTATE_DOWN;
                    break;
                case AcqState.ROTATE_READY_RETRACT://angle at which it is safe to extend the rollers
                    setAcquiringMotor(0);
                    //acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                    acqShortPnu.set(!ACQ_SHORT_PNU_EXTENDED);
                    acquisitionState = AcqState.ROTATE_UP;
                    break;
                case AcqState.ACQUIRING://Rollers are running and we are getting a ball
                        setAcquiringMotor(INTAKE_ROLLER_SPEED);//Turns rollers on
                        if(!ballDetector.get()){
                            acquisitionState = AcqState.ACQUIRED;
                            log.logMessage("Ball Detected!");
                        }
                    break;
                case AcqState.ACQUIRED://limit switch has been pressed - short cylinder retracts
                    setAcquiringMotor(0);
                    //acqShortPnu.set(!ACQ_SHORT_PNU_EXTENDED);//Ball can't escape
                    break;
                case AcqState.EJECT_BALL://ball is being ejected from robot through rollers
                    acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                    acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);//Ball rollers right on out
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
                    setAcquiringMotor(0);
                    break;
            }
            if(!IO.USE_PWM_CABLES){
                rotatingMotor.setX(rotationSpeed);
            }else{
                rotatingMotorPWM.set(rotationSpeed);
            }
            if(Timer.getFPGATimestamp() - LOG_EVERY >= lastLogTime && ds.isEnabled()){
                lastLogTime = Timer.getFPGATimestamp();
                logFile();
            }
        }
    }
    
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
    
    private void logFile(){
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
            switch(state){
                case AcqState.ACQUIRING:
                    wantedShooterAngle = 120;
                    acquisitionState = AcqState.ROTATE_DOWN;
                    break;
                case AcqState.SAFE_STATE:
                    wantedShooterAngle = 0;
                    acquisitionState = AcqState.ROTATE_UP;
                    break;
                case AcqState.READY_TO_SHOOT:
                    setPreset(MID_SHOOTER_PRESET);
                    break;
                case AcqState.EJECT_BALL:
                    wantedShooterAngle = 120;//Acquiring
                    acquisitionState = AcqState.ROTATE_DOWN;
                    break;
            }
    }

    /**
     * Sets the wanted angle of the shooter.
     * 0 - straight up and down
     * 120 - acquiring
     * @param preset 
     */
    public void setPreset(int preset){
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
                wantedShooterAngle = 0;
        }
    }
    
    /**
     * Sets the wanted shooting angle of the robot. And rotates the shooter
     * @param angle - wanted angle in degrees.
     * 0 - straight up
     * 120 - acquiring
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
    
    public void addOffset(int offset){
        wantedShooterAngle += offset;
        if(wantedShooterAngle < 0){
            wantedShooterAngle = 0;
        }else if(wantedShooterAngle > 120){
            wantedShooterAngle = 120;
        }
        if(offset < 0){
            acquisitionState = AcqState.ROTATE_UP;
        }else{
            acquisitionState = AcqState.ROTATE_DOWN;
        }
        wantedState = AcqState.READY_TO_SHOOT;
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
