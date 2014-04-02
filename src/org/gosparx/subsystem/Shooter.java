package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.gosparx.IO;
import org.gosparx.sensors.PotentiometerData;
import org.gosparx.util.Logger;

/**
 * @author Alex
 * Stores the state of the shooter and controls the winch and latch for the 
 * shooter
 */
public class Shooter extends GenericSubsystem{
    
     /**
     * The timeout for winding the cable onto the winch, in seconds.
     */ 
    private static final double WIND_TIMEOUT = 5;//MAY BE MORE
    
    /**
     * The inches the pot travels per volt change.
     */
    private static final double INCHES_PER_VOLT = -12.566370614359172953850573533118;//2 inch wheel
    
    /**
     * The # of inches to wind and unwind the cable when shooting.
     */
    //TODO: Confirm value
    public static final double MAX_UNWIND_INCHES = 14;
    
    public static final double MIN_UNWIND_INCHES = 3;
    
    /**
     * The timeout in seconds for unwinding the cable on the winch
     */ 
    private static final double UNWIND_TIMEOUT = 5;
    
     /**
     * The motor speed to pull back the winch at.
     */
    private static final double WINCH_SPEED = -1.00;
    
    /**
     * The boolean constant if the latch is engaged.
     */
    private static final boolean LATCH_ENGAGED = false;
    
    /**
     * The boolean constant if the latch is disengaged.
     */
    private static final boolean LATCH_DISENGAGED = !LATCH_ENGAGED;
    
    /**
     * How long in seconds between shooting the ball and starting to pull the
     * winch back again.
     */ 
    private static final double TIME_BETWEEN_SHOTS = 1.00;
    
    /**
     * 
     */
    private static final double TIME_TO_TAKE_PICTURE = 0.01;
    
    /**
     * The time from when we fire to the time we capture and save an image
     * The amount of time that the pnu latch needs to completely latch the 
     * shooter
     */
    private static final double LATCH_TIME = 0.75;
    
    /**
     * The Potentiometer for the winch. 
     */
    private AnalogPotentiometer winchPot;
    
    /**
     * The motor controller for the right side of the winch.
     */
    private Talon rightWinchMotor;

    /**
     * The motor controller for the left side of the winch
     */
    private Talon leftWinchMotor;
    
    /**
     * The limit switch for the winch latch.
     */ 
    private DigitalInput latchSwitch;
    
    /**
     * The current state of the shooter.
     */ 
    private int shooterState;
    
    /**
     * The Solenoid of latch.
     */ 
    private Solenoid latch;

    /**
     * The speed that the winch is set at the end of the execute() loop.
     */
    private double wantedWinchSpeed;
    
    /**
     * The time of the last shot. Used to tell when we can start pulling the
     * winch back after the shot.
     */ 
    private double lastShotTime;
    
    /**
     * The shooter used for the singleton model.
     */
    private static Shooter shooter;
        
    /**
     * The PotentiometerData for the pot that is on the winch.
     */
    private PotentiometerData potData;
    
    /**
     * The last FPGA time when we started winding the winch.
     */ 
    private double lastWindTime;
    
    /**
     * The last FPGA time we started unwinding the winch.
     */
    private double lastUnwindTime;
    
    /**
     * The value of the limit switch
     */
    private boolean limitSwitchValue;

    /**
     * Determines if the camera should save a picture after shooting
     */
    private boolean gotLastShot = false;
    /**
     * The subsystem name that all of the components are listed under in the
     * livewindow.
     */ 
    private String subsystemName = "Shooter";
    
    private boolean lastShotWound = false;
    
    private double inchesUnwound;
    
    private double potInches = 0;
    
    /**
     * Returns an instance of a shooter. Used in the singleton model.
     */
    public static Shooter getInstance(){
        if(shooter == null){
            shooter = new Shooter();
        }
        return shooter;
    }
    
    /**
     * Creates a new Shooter object.
     */
    private Shooter(){
        super(Logger.SUB_SHOOTER, Thread.NORM_PRIORITY);
    }

    /**
     * Initializes everything.
     */ 
    public void init() {
        rightWinchMotor = new Talon(IO.DEFAULT_SLOT, IO.PWM_WINCH);
        latchSwitch = new DigitalInput(IO.DEFAULT_SLOT, IO.LATCH_LIMIT_SWITCH_CHAN);
        latch = new Solenoid(IO.DEFAULT_SLOT, IO.LATCH_CHAN);
        latch.set(LATCH_ENGAGED);
        shooterState = State.STANDBY;
        winchPot = new AnalogPotentiometer(IO.WINCH_POT_CHAN);
        potData = new PotentiometerData(winchPot, INCHES_PER_VOLT);
        leftWinchMotor = new Talon(IO.DEFAULT_SLOT, IO.PWM_WINCH_2);
        if(!latchSwitch.get()){
            potData.reset();
        }
    }

    /**
     * Loops. 
     */ 
    public void execute() throws Exception {
        potInches = potData.getInches();
        wantedWinchSpeed = 0;
        limitSwitchValue = !latchSwitch.get();
        switch(shooterState){
            // Disengauges the latch and then sets the lastShotTime to the 
            // current FPGA time. Then sets the State to SHOOTER_COOLDOWN
            case State.SHOOT:
                log.logMessage("Shooting");
                latch.set(LATCH_DISENGAGED);
                lastShotTime = Timer.getFPGATimestamp();
                shooterState = State.SHOOTER_COOLDOWN;
                lastShotWound = potInches < MAX_UNWIND_INCHES/2;
                log.logMessage("lastShotWound: " + lastShotWound + " potInches: " + potInches);
                break;
            // Does nothing
            case State.STANDBY:
                latch.set(LATCH_ENGAGED);
                wantedWinchSpeed = 0;
                break;
            // Does nothing for TIME_BETWEEN_SHOTS seconds after the last shot.
            // Then starts retracting the winch.
            case State.SHOOTER_COOLDOWN:
                if (Timer.getFPGATimestamp() - lastShotTime >= TIME_TO_TAKE_PICTURE && !gotLastShot) {
                    Vision.getInstance().setSaveImage();
                    log.logMessage("Saved Image");
                    gotLastShot = true;
                } else if (Timer.getFPGATimestamp() - lastShotTime <= TIME_TO_TAKE_PICTURE) {
                    gotLastShot = false;
                }
                if (Timer.getFPGATimestamp() - lastShotTime >= TIME_BETWEEN_SHOTS) {
                    log.logMessage("Shooter Cooldown Complete");
                    if (limitSwitchValue || lastShotWound) {
                        log.logMessage("Not auto winding because of limitSwitch:" + limitSwitchValue
                                + " lastShotWound:"+lastShotWound);
                        shooterState = State.STANDBY;
                    } else {
                        log.logMessage("Trying to set home");
                        shooterState = State.SET_HOME;
                    }
                }
                break;
            case State.SET_HOME:
                latch.set(LATCH_DISENGAGED);
                wantedWinchSpeed = WINCH_SPEED;
                if(limitSwitchValue){
                    log.logMessage("latch has been triggered");
                    shooterState = State.HOLD_WINCH;
                    potData.reset();
                    lastUnwindTime = Timer.getFPGATimestamp();
                }
                break;
            case State.HOLD_WINCH:
                latch.set(LATCH_ENGAGED);
                wantedWinchSpeed = WINCH_SPEED;
                if(Timer.getFPGATimestamp() - lastUnwindTime >= LATCH_TIME){
                    lastUnwindTime = Timer.getFPGATimestamp();
                    log.logMessage("Unwinding Winch");
                    wantedWinchSpeed = 0;
                    shooterState = State.UNWINDING;
                }
                break;
            case State.UNWINDING:
                wantedWinchSpeed = -WINCH_SPEED;
                if((Timer.getFPGATimestamp() - lastUnwindTime >= UNWIND_TIMEOUT) || potInches >= MAX_UNWIND_INCHES){
                    log.logMessage("Unwinding complete");
                    wantedWinchSpeed = 0;
                    shooterState = State.STANDBY;
                }
                break;
           case State.SHOOTER_WINDING:
                wantedWinchSpeed = WINCH_SPEED;
                if (potInches <= inchesUnwound) {
                    log.logMessage("Winding Complete");
                    wantedWinchSpeed = 0;
                    shooterState = State.STANDBY;
                }
                break;
            case State.SHOOTER_UNWINDING:
                wantedWinchSpeed = -WINCH_SPEED;
                if(potInches >= inchesUnwound){
                    log.logMessage("Unwinding Complete");
                    wantedWinchSpeed = 0;
                    shooterState = State.STANDBY;
                }
                break;
            default:
                log.logError("Unknown Shooter state: " + shooterState);
                break;
        }
        rightWinchMotor.set(wantedWinchSpeed);
        leftWinchMotor.set(-wantedWinchSpeed);
    }
    
    /**
     * Sets the shooter mode to the wantedState. Use the State classes constants.
     * @param wantedState - the desired state.
     */
    public void setMode(int wantedState){
        shooterState = wantedState;
        log.logMessage("NEW STATE HAS BEEN SET TO: " + State.getState(wantedState));
    }
    
    public void setAdjustSlack(double inchesUnwound){
        this.inchesUnwound = inchesUnwound;
        if(inchesUnwound < potInches){
            shooterState = State.SHOOTER_WINDING;
        }else{
            shooterState = State.SHOOTER_UNWINDING;
        }
    }
    
    /**
     * Sets the state of the Shooter to State.Shoot if the current state of the robot is standby.
     * @return if the shooter attempted to shoot
     */ 
    public boolean shoot(){
       if(shooterState == State.STANDBY && (Acquisitions.getInstance().readyToShoot() || ds.isOperatorControl())){
           shooterState = State.SHOOT;
           log.logMessage("Shooting");
           return true;
       }
       log.logMessage("Attempted to shoot, but could not");
       return false;
    }
    
    /**
     * Initializes and adds all of the components to the livewindow.
     */ 
    public void liveWindow() {
        LiveWindow.addActuator(subsystemName, "Winch 1", rightWinchMotor);
        LiveWindow.addSensor(subsystemName, "Winch Pot", winchPot);
        LiveWindow.addActuator(subsystemName, "Winch 2", leftWinchMotor);
        LiveWindow.addActuator(subsystemName, "Fire", latch);
        LiveWindow.addSensor(subsystemName, "Winch Stop Limit", latchSwitch);
    }

    public int sleepTime() {
        return 20;
    }

    public void logInfo() {
        log.logMessage("Current State: " + State.getState(shooterState));
        log.logMessage("Pot Dist: " + potData.getInches() + " Pot Voltage: " + winchPot.get());
        log.logMessage("Wanted Winch Speed: " + wantedWinchSpeed);
    }
    
    public boolean isLastCommandDone(){
            return shooterState == State.STANDBY;
    }
    
    /**
     * A class used for storing possible states of the Shooter.
     * 
     * SHOOT - Shoots the ball
 RETRACT - disengages the latch and then sets to WINDIND
 STANDBY - Does nothing
 SHOOTER_COOLDOWN - Waits TIME_BETWEEN_SHOTS seconds after the shot, then
                    starts winding
 SET_HOME - Drives the winch all the way back, sets the pot to 0, and then
            engages the latch and unwinds the winch
 UNWIND_WINCH - Engages the latch then sets the state to UNSHOOTER_WINDING
 UNSHOOTER_WINDING - Unwinds the winch until the pot reaches INCHES_TO_WIND, then
             Goes to Standby
 SHOOTER_WINDING - Winds the winch until it reaches 0 or 
     */ 
    public static class State{
        public static final int SHOOT = 1;
        public static final int STANDBY = 3;
        public static final int SHOOTER_COOLDOWN = 4;
        public static final int SET_HOME = 5;
        public static final int HOLD_WINCH = 6;
        public static final int UNWINDING = 7;
        public static final int SHOOTER_WINDING = 8;
        public static final int SHOOTER_UNWINDING = 9;
        
        /**
         * Returns a string version of the state.
         */
        public static String getState(int state){
            switch(state){
                case STANDBY:
                    return "Standby";
                case SHOOT:
                    return "Shooting";
                case SHOOTER_COOLDOWN:
                    return "Shooter is cooling down";
                case SET_HOME:
                    return "Setting home";
                case HOLD_WINCH:
                    return "Holding Winch";
                case UNWINDING:
                    return "Unwinding Winch";
                case SHOOTER_WINDING:
                    return "Winding Winch";
                case SHOOTER_UNWINDING:
                    return "Shooter Unwinding";
            }
            return "UNKOWN MODE: " + state;
        }
    }
}
