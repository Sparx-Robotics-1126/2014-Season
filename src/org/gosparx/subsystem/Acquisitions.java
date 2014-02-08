package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.gosparx.Autonomous;
import org.gosparx.IO;
import org.gosparx.sensors.EncoderData;

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
    
    /**
     * Used to control the intake rollers
     * Bag Motor/Fisher Price driven with a 5/1 reduction (motor/output)
     */
    private CANJaguar acqRoller;
    
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
    private final static double DEGREES_PER_TICK = 0.0;//TODO: MATH
    
    /**
     * The angle at which it is legal for the acquisition rollers to extend without breaking rules
     */
    private final static int ACQ_ROLLER_ALLOWED_TO_EXTEND = 110;//TODO: CHECK
    
    /**
     * The name of this subsystem.
     * Used in live window to setup all the motors/sensors/solenoid to be grouped with with class
     */
    private final static String subsystemName = "Acquisitions";
   
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
    private int wantedShooterAngle = 0;
    
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
        super("Acq", Thread.NORM_PRIORITY);
    }
    
    /**
     * Initiates all of the motors/sensors/solenoids.
     * Sets the short cylinder to its default position.
     */
    public void init() {
        try {
            rotatingMotor = new CANJaguar(IO.CAN_ADRESS_PIVOT);
            acqRoller = new CANJaguar(IO.CAN_ADRESS_ACQ);
        } catch (CANTimeoutException ex) {
            log.logError("CANBus Timeout it Acquisitions2 init()");
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
        acquisitionState = AcqState.SAFE_STATE;//default state
    }

    /**
     * Main run method. Is called through genericSubsystem.
     * @throws Exception - if thrown then the thread will try to restart itself
     */
    public void execute() throws Exception {
        while(!ds.isTest()){//motors can't be given values during test mode, OR IT DOSEN'T WORK
            rotateEncoderData.calculateSpeed();//Calculates the distance and speed of the encoder
            if(currentStatePosition < 50){
            System.out.println("Wanted State: " + wantedState + " Current State: " + acquisitionState + " Command: " + AcqState.getStateName(acquisitionState));
            }
            switch(acquisitionState){                
                case AcqState.ROTATE_UP://rotate shooter up
                    if(wantedShooterAngle == 0 && !upperLimit.get()){//straight up and down
                        rotationSpeed = 0;
                        findNewCase();
                    }else if(wantedShooterAngle >= rotateEncoderData.getDistance()){
                        rotationSpeed = 0;
                        findNewCase();
                    }else{
                        rotationSpeed = 0.2;//MAY WANT TO RAMP
                    }
                    if(!upperLimit.get()){
                        rotationSpeed = 0;
                        log.logMessage("Upper Limit has been tripped, unknown position");
                    }
                    break;
                case AcqState.ROTATE_DOWN://rotate shooter down
                    if(wantedShooterAngle == 120 && !lowerLimit.get()){//limit Switch reads false when touched
                        rotationSpeed = 0;
                        findNewCase();
                    }else if(wantedShooterAngle <= rotateEncoderData.getDistance()){
                        rotationSpeed = 0;
                        findNewCase();
                    }else{
                        rotationSpeed = -0.2;//MAY WANT TO RAMP
                    }
                    if(!lowerLimit.get()){
                        rotationSpeed = 0;
                        log.logMessage("Lower Limit has been tripped, unknown position");
                    }
                    break;
                case AcqState.ROTATE_READY_TO_EXTEND://angle at which it is safe to extend the rollers
                        acqLongPnu.set(ACQ_LONG_PNU_EXTENDED);
                        findNewCase();
                    break;
                case AcqState.ACQUIRING://Rollers are running and we are getting a ball
                    setAcquiringMotor(INTAKE_ROLLER_SPEED);//Turns rollers on
                    findNewCase();
                    break;
                case AcqState.ACQUIRED://limit switch has been pressed - short cylinder retracts
                    if(ballDetector.get()){
                        setAcquiringMotor(0);
                        acqShortPnu.set(!ACQ_SHORT_PNU_EXTENDED);//Ball can't escape
                        findNewCase();
                    }
                    break;
                case AcqState.EJECT_BALL://ball is being ejected from robot through rollers
                    acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                    acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);//Ball rollers right on out
                    findNewCase();
                    break;
                case AcqState.READY_TO_RETRACT://The maximum angle to be at before an over 5' penalty
                    acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                    acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                    findNewCase();
                    break;
                case AcqState.READY_TO_SHOOT://Rollers are out of the way, Shooting angle is set
                    acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                    findNewCase();
                    break;
                case AcqState.SAFE_STATE://Shooter is in the robots perimeter
                    findNewCase();
                    break;
                case AcqState.OFF_STATE://Something has gone wrong. All motors are set to 0.0
                    rotationSpeed = 0;
                    acqShortPnu.set(ACQ_SHORT_PNU_EXTENDED);
                    acqLongPnu.set(!ACQ_LONG_PNU_EXTENDED);
                    setAcquiringMotor(0);
                    break;
            }
            rotatingMotor.setX(rotationSpeed);
        }
    }
    
    private void setAcquiringMotor(double value){
        try {
            acqRoller.setX(value * -1);//motor runs backwards. (silly motors)
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
    
    /**
     * If the current state is in ready to shoot then it is ok to shoot
     * @return if the acquisitions system is ready to shoot or not 
     */
    public boolean readyToShoot(){
        return(acquisitionState == AcqState.READY_TO_SHOOT);
    }
    
    private int currentStatePosition = 0;
    private void findNewCase(){
        rotateEncoderData.calculateSpeed();
        switch(wantedState){
            case AcqState.ACQUIRING:
                switch(currentStatePosition){
                    case 1:
                        acquisitionState = AcqState.ROTATE_DOWN;
                        break;
                    case 2:
                        acquisitionState = AcqState.ROTATE_READY_TO_EXTEND;
                        break;
                    case 3:
                        acquisitionState = AcqState.ACQUIRING;
                        break;
                    case 4:
                        acquisitionState = AcqState.ROTATE_DOWN;
                        break;
                }
                break;
            case AcqState.READY_TO_SHOOT:
                switch(currentStatePosition){
                    case 1:
                        if(rotateEncoderData.getDistance() < 5){
                            acquisitionState = AcqState.ROTATE_DOWN;
                        }else{
                            acquisitionState = AcqState.ROTATE_UP;
                        }
                    case 2:
                        acquisitionState = AcqState.READY_TO_SHOOT;
                        break;
                    case 3:
                        acquisitionState = AcqState.READY_TO_SHOOT;
                        break;
                }
                break;
            case AcqState.EJECT_BALL:
                switch(currentStatePosition){
                    case 1:
                        acquisitionState = AcqState.ROTATE_DOWN;
                        break;
                    case 2:
                        acquisitionState = AcqState.EJECT_BALL;
                        break;
                }
                break;
            case AcqState.SAFE_STATE:
                switch(currentStatePosition){
                    case 1:
                        acquisitionState = AcqState.ROTATE_UP;
                        break;
                    case 2:
                        acquisitionState = AcqState.SAFE_STATE;
                }
                break;
            case AcqState.OFF_STATE:
                acquisitionState = AcqState.OFF_STATE;
                break;
            default:
                currentStatePosition = 100;
                break;
        }
    }
    
    /**
     * The state you want the system to enter
     * @param state - get from AcqsitionState 
     */
    public void setMode(int state){
        wantedState = state;
        currentStatePosition = 1;
        findNewCase();
    }
    
    /**
     * Is called right after init().
     * This groups all of the motors/sensors/solenoid together.
     * Sets up the live window screen used in test mode to control each system manually.
     */
    public void liveWindow() {
        LiveWindow.addActuator(subsystemName, "Pivot", rotatingMotor);
        LiveWindow.addActuator(subsystemName, "Acquisitions", acqRoller);
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
        public static final int READY_TO_SHOOT = 8;
        public static final int SAFE_STATE = 9;
        public static final int OFF_STATE = 10;
        
        public static String getStateName(int state){
            switch(state){
                case ROTATE_UP:
                    return "Rotating Up";
                case ROTATE_DOWN:
                    return "Rotating Down";
                case ROTATE_READY_TO_EXTEND:
                    return "Rotate REady to Extend";
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
