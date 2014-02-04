package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import org.gosparx.IO;
import org.gosparx.sensors.EncoderData;
import org.gosparx.util.Logger;

/**
 * @author Alex
 */
public class Acquisitions extends GenericSubsystem{
    
    /**
     * The timeout in seconds for pivoting to acquiring position.
     */ 
    private static final double PIVOT_TIMEOUT = 15;
    
    /**
     * The timeout in seconds for setting the home position.
     */ 
    private static final double SET_HOME_TIMEOUT = 10;
    
     /**
     * The time in seconds to continue running the acquisitions motors after the 
     * limit switch has been triggered.
     */ 
    private static final double AQUIRE_TIME                                       = .5;
    
    /**
     * The time in seconds to continue running the acquisitions motors in
     * reverse the exiting ball has triggered the limit switch.
     */ 
    private static final double RELEASE_TIME                                     = .75;
    
     /**
     * The % max speed to run the motors when pivoting.
     */
    private static final double PIVOT_TURN_SPEED                                 = .25;
    
    /**
     * The % max speed to run the motors when acquiring a ball.
     */ 
    private static final double ROLLER_SPEED                                       = 1;
     /**
     * The solenoid value if the acquisitions system is down.
     */    
    private static final boolean ACQ_DOWN                               = true;
    
    /**
     * The solenoid value if the acquisitions system is up.
     */
    private static final boolean ACQ_UP                                 = false;
        
    /**
     * The amount of degrees plus or minus we see as good when pivoting.
     */ 
    private static final double PIVOT_TOLERANCE = 2.5;
    
    /**
     * The angle of shooter when it is in truss passing position
     * TODO: Get proper angle
     */
    public static final double MODE_TRUSS = 0.0;
    
    /**
     * The angle of the shooter when it is in shooter mode.
     * TODO: Get proper angle
     */
    public static final double MODE_SHOOT = 0.0;
    
    /**
     * The angle of the shooter when the robot is in aquire mode.
     * TODO: Get proper angle
     */ 
    public static final double MODE_ACQUIRE = 0.0;
    
    /**
     * The degrees that the shooter rotates per tick of the encoder
     */
    private static final double DEGREES_PER_TICK                          = 0.0;
    
    /**
     * The solenoid that toggles if the acquisitions rollers are able to hit the 
     * ball.
     */ 
    private Solenoid acqToggle;
     
    /**
     * The limit switch on the acquisitions system. It detects if a ball has 
     * been sucked in.
     */ 
    private DigitalInput acqLimitSwitch;
    
    /**
     * The Jaguar that controls the pivoting system.
     */  
    private CANJaguar pivotMotor;
    
    /**
     * The Jaguar that controls the acquisitions system.
     */
    private CANJaguar acqMotor;
        
    /**
     * The limit switch that detects if the shooter is in "home" position, all
     * the way up.
     */
    private DigitalInput shooterSafeModeSwitch;
    
    /**
     * The encoder that tracks the pivoting motion.
     */ 
    private Encoder pivotEncoder;
    
    /**
     * The encoder data for the pivotEncoder.
     */ 
    private EncoderData pivotEncoderData;
    
    /**
     * The limit switch that detects if the shooter is in aquire mode.
     */ 
    private DigitalInput shooterAcqModeSwitch;
    
    /**
     * The current state of the acquisitions system.
     */ 
    private int acquisitionsState;
    
    
    /**
     * The speed to set the pivot motors at.
     */ 
    private double wantedSpeedPivot;
    
    /**
     * The speed to set the acquisitions motors at.
     */ 
    private double wantedSpeedAcq;
    
    /**
     * Stores if the limit switch has been triggered when releasing the ball.
     */ 
    private boolean hasHitSwitchOnRelease;
    
    /**
     * Stores if the limit switch has been triggered when acquiring the ball.
     */ 
    private boolean hasHitSwitchOnAcquire;
    
    /**
     * The FPGA time when the limit switch was hit when releasing the ball.
     */
    private double timeHitRelease;
    
    /**
     * The FPGA time when the limit switch was hit when acquiring the ball.
     */ 
    private double timeHitEntering;
    
    /**
     * The solenoid for keeping the acquisitions within the frame perimeter at 
     * the start of the match.
     */ 
    private Solenoid keepInFrame;
    
    /**
     * The time we attempted to start acquiring.
     */ 
    private double startTimeAquire;
    
    /**
     * The time we attempted to start releasing the ball.
     */
    private double startTimeRelease;
    
    /**
     * The FPGA time when we started to attempt to set the home.
     */ 
    private double startTimeSetHome = -1;
    
    /**
     * A private Acquisitions used for the singleton model.
     */
    private static Acquisitions acq;
    
    /**
     * The desired angle for the shooter to be positioned at. Use constants when
     * possible.
     */ 
    private double wantedAngle;
    
    /**
     * Returns the singleton Acquisitions.
     */
    public static Acquisitions getInstance(){
        if(acq == null){
            acq = new Acquisitions();
        }
        return acq;
    }
    /**
     * Creates a new Acquisitions.
     */
    private Acquisitions(){
        super(Logger.SUB_ACQUISITIONS, Thread.NORM_PRIORITY);
    }
    
    /**
     * Initializes everything.
     */ 
    public void init() {
        try {
            pivotMotor = new CANJaguar(IO.CAN_ADRESS_PIVOT);
            acqMotor = new CANJaguar(IO.CAN_ADRESS_ACQ);
        } catch (CANTimeoutException ex) {
            log.logError("CANBus Timeout it Acquisitions init()");
        }
        acqToggle = new Solenoid(IO.ACQ_TOGGLE_CHAN);
        acqToggle.set(ACQ_UP);
        acqLimitSwitch = new DigitalInput(IO.ACQ_SWITCH_CHAN);
        shooterSafeModeSwitch = new DigitalInput(IO.SHOOTER_SAFE_MODE_CHAN);
        pivotEncoder = new Encoder(IO.DEFAULT_SLOT, IO.PIVOT_ENCODER_CHAN_1, IO.DEFAULT_SLOT, IO.PIVOT_ENCODER_CHAN_2);
        pivotEncoder.setDistancePerPulse(DEGREES_PER_TICK);
        pivotEncoderData = new EncoderData(pivotEncoder, DEGREES_PER_TICK);
        shooterAcqModeSwitch = new DigitalInput(IO.SHOOTER_ACQ_MODE_CHAN);
        keepInFrame = new Solenoid(IO.KEEP_IN_FRAME_CHAN);
        keepInFrame.set(true);
        acquisitionsState = State.SETTING_HOME;
    }

    /**
     * Loops.
     */ 
    public void execute() throws Exception {
        while(true){
            wantedSpeedAcq = 0;
            wantedSpeedPivot = 0;
            switch(acquisitionsState){
                case State.STANDBY:
                    wantedSpeedAcq = 0;
                    wantedSpeedPivot = 0;
                    break;
                case State.ACQUIRING:
                    if(!shooterAcqModeSwitch.get()  && !(Timer.getFPGATimestamp() - startTimeAquire > PIVOT_TIMEOUT)){
                        wantedSpeedPivot = PIVOT_TURN_SPEED;
                    }else{
                        wantedSpeedPivot = 0;
                        wantedSpeedAcq = ROLLER_SPEED;
                        acqToggle.set(ACQ_DOWN);
                    }
                    if(acqLimitSwitch.get()){
                        hasHitSwitchOnAcquire = true;
                        timeHitEntering = Timer.getFPGATimestamp();
                    }
                    if(hasHitSwitchOnAcquire && Timer.getFPGATimestamp() - timeHitEntering > AQUIRE_TIME){
                        wantedSpeedAcq = 0;
                        acqToggle.set(ACQ_UP);
                        hasHitSwitchOnAcquire = false;
                        acquisitionsState = State.STANDBY;
                    }
                    break;
                case State.PIVOTING:
                    wantedSpeedPivot = (pivotEncoderData.getDistance() > wantedAngle) ? -PIVOT_TURN_SPEED : PIVOT_TURN_SPEED;
                    if(Math.abs(pivotEncoderData.getDistance() - wantedAngle) <= PIVOT_TOLERANCE){
                        wantedSpeedPivot = 0;
                        acquisitionsState = State.STANDBY;
                    }
                    break;
                case State.ASSISTING:
                    if(!shooterAcqModeSwitch.get() && !(Timer.getFPGATimestamp() - startTimeRelease > PIVOT_TIMEOUT)){
                        wantedSpeedPivot = PIVOT_TURN_SPEED;
                    }else{
                        wantedSpeedPivot = 0;
                        wantedSpeedAcq = -ROLLER_SPEED;
                        acqToggle.set(ACQ_UP);
                    }
                    if(acqLimitSwitch.get()){
                        hasHitSwitchOnRelease = true;
                        timeHitRelease = Timer.getFPGATimestamp();
                    }
                    if(hasHitSwitchOnRelease && Timer.getFPGATimestamp() - timeHitRelease > RELEASE_TIME){
                        wantedSpeedAcq = 0;
                        acqToggle.set(ACQ_DOWN);
                        hasHitSwitchOnRelease = false;
                        acquisitionsState = State.STANDBY;
                    }
                    break;
                case State.SETTING_HOME:
                    if(ds.isEnabled() && startTimeSetHome == -1){
                        startTimeSetHome = Timer.getFPGATimestamp();
                    }
                    wantedSpeedPivot = -.1;
                    if(shooterSafeModeSwitch.get() || Timer.getFPGATimestamp() - startTimeSetHome > SET_HOME_TIMEOUT){
                        wantedSpeedPivot = 0;
                        pivotEncoderData.reset();
                        acquisitionsState = State.STANDBY;
                    }
                    break;
                default:
                    log.logError("Unknown state for Acquisitions: " + acquisitionsState);
            }

            if(ds.isEnabled() && keepInFrame.get()){
                keepInFrame.set(false);
            }
            acqMotor.setX(wantedSpeedAcq);
            pivotMotor.setX(wantedSpeedPivot);
            
            if(Timer.getFPGATimestamp() - lastLogTime >= LOG_EVERY){
                log.logMessage("Current State: " + State.getState(acquisitionsState));
                log.logMessage("Pivot Encoder Dist: " + pivotEncoderData.getDistance());
                log.logMessage("Pivot Motor Wanted Speed: " + wantedSpeedPivot + " Wanted Acq Speed: " + wantedSpeedAcq);
                log.logMessage("Wanted Shooter Angle: " + wantedAngle);
                log.logMessage("Is Ready to Shoot: " + isReadyToShoot());
                lastLogTime = Timer.getFPGATimestamp();
            }
        }
    }
    
    /**
     * Sets the start acquire time and sets the state to State.ACQUIRING.
     */ 
    public void acquire(){
        startTimeAquire = Timer.getFPGATimestamp();
        acquisitionsState = State.ACQUIRING;
    }
    
    /**
     * Sets the start release time and sets State.ASSISTING.
     */ 
    public void release(){
        startTimeRelease = Timer.getFPGATimestamp();
        acquisitionsState = State.ASSISTING;
    }
    
    public void pivot(double degrees){
        wantedAngle = degrees;
        acquisitionsState = State.PIVOTING;
    }
    
    public boolean isReadyToShoot(){
        return (acquisitionsState == State.STANDBY && Math.abs(pivotEncoderData.getDistance() - MODE_SHOOT) <= PIVOT_TOLERANCE) || (acquisitionsState == State.STANDBY && Math.abs(pivotEncoderData.getDistance() - MODE_TRUSS) <= PIVOT_TOLERANCE);
    }
    /**
     * A class for storing the state of acquisitions.
     */ 
    public static class State{
        public static final int STANDBY = 1;
        public static final int ACQUIRING = 2;
        public static final int PIVOTING = 3;
        public static final int ASSISTING = 4;
        public static final int SETTING_HOME = 5;
        
        /**
         * Returns a string version of the state.
         */
        public static String getState(int state){
            switch(state){
                case STANDBY:
                    return "Standby";
                case ACQUIRING:
                    return "Acquiring";
                case PIVOTING:
                    return "Pivoting";
                case ASSISTING:
                    return "Assisting";
                case SETTING_HOME:
                    return "Setting home";
            }
            return "UNKOWN MODE: " + state;
        }
    }
}
