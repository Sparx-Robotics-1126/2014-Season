package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
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
    private static final double WIND_TIMEOUT = 2;//MAY BE MORE
    
    /**
     * The Diameter of the cylinder we are wrapping the cable around.
     */ 
    private static final double DIAMETER_CYLINDER = 2;
    
    /**
     * The number of turns in the potentiometer.
     */ 
    private static final double POT_TURNS = 10;
    
    /**
     * The inches the pot travels per volt change.
     */
    private static final double INCHES_PER_VOLT = (DIAMETER_CYLINDER * Math.PI) / (5 / POT_TURNS);
    
    /**
     * The # of inches to wind and unwind the cable when shooting.
     */
    //TODO: Confirm value
    private static final double INCHES_TO_WIND = 15;
    
    /**
     * The timeout in seconds for unwinding the cable on the winch
     */ 
    private static final double UNWIND_TIMEOUT = 2;
    
     /**
     * The motor speed to pull back the winch at.
     */
    private static final double WINCH_SPEED = 0.5;
    
    /**
     * The boolean constant if the latch is engaged.
     */
    private static final boolean LATCH_ENGAGED = true;
    
    /**
     * The boolean constant if the latch is disengaged.
     */
    private static final boolean LATCH_DISENGAGED = !LATCH_ENGAGED;
    
    /**
     * How long in seconds between shooting the ball and starting to pull the
     * winch back again.
     */ 
    private static final double TIME_BETWEEN_SHOTS = .5;
    
    /**
     * The Potentiometer for the winch. 
     */
    private AnalogPotentiometer winchPot;
    
    /**
     * The motor controller for the winch.
     */
    private CANJaguar winchMotor;
    private Jaguar winchMotorPWM;
    
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
        if(!IO.USE_PWM_CABLES){
            try {
                winchMotor = new CANJaguar(IO.CAN_ADRESS_WINCH);
            } catch (CANTimeoutException ex) {
                log.logError("CANBus timeout in Shooter init()");
            }
        }else{
            winchMotorPWM = new Jaguar(IO.DEFAULT_SLOT, IO.PWM_WINCH);
        }
        latchSwitch = new DigitalInput(IO.LATCH_SWITCH_CHAN);
        latch = new Solenoid(IO.LATCH_CHAN);
        latch.set(LATCH_ENGAGED);
        shooterState = State.STANDBY;
        winchPot = new AnalogPotentiometer(IO.WINCH_POT_CHAN);
        potData = new PotentiometerData(winchPot, INCHES_PER_VOLT);
    }

    /**
     * Loops. 
     */ 
    public void execute() throws Exception {
        while(!ds.isTest()){
            wantedWinchSpeed = 0;
            switch(shooterState){
                // Disengauges the latch and then sets the lastShotTime to the 
                // current FPGA time. Then sets the State to SHOOTER_COOLDOWN
                case State.SHOOT:
                    latch.set(LATCH_DISENGAGED);
                    lastShotTime = Timer.getFPGATimestamp();
                    shooterState = State.SHOOTER_COOLDOWN;
                    break;
                // Retracts the winch until the limit switch is hit
                case State.RETRACT:
                    latch.set(LATCH_DISENGAGED);
                    lastWindTime = Timer.getFPGATimestamp();
                    shooterState = State.WINDING;
                    break;
                // Does nothing
                case State.STANDBY:
                    wantedWinchSpeed = 0;
                    break;
                // Does nothing for TIME_BETWEEN_SHOTS seconds after the last shot.
                // Then starts retracting the winch.
                case State.SHOOTER_COOLDOWN:
                    if(Timer.getFPGATimestamp() - lastShotTime >= TIME_BETWEEN_SHOTS){
                        shooterState = State.RETRACT;
                    } 
                    break;
                case State.SET_HOME:
                    wantedWinchSpeed = WINCH_SPEED;
                    if(latchSwitch.get()){
                        wantedWinchSpeed = 0;
                        shooterState = State.UNWIND_WINCH;
                        potData.reset();
                    }
                    break;
                case State.UNWIND_WINCH:
                    latch.set(LATCH_ENGAGED);
                    lastUnwindTime = Timer.getFPGATimestamp();
                    shooterState = State.UNWINDING;
                    break;
                case State.UNWINDING:
                    wantedWinchSpeed = -WINCH_SPEED;
                    if((Timer.getFPGATimestamp() - lastUnwindTime >= UNWIND_TIMEOUT) || potData.getInches() >= INCHES_TO_WIND){
                        wantedWinchSpeed = 0;
                        shooterState = State.STANDBY;
                    }
                    break;
               case State.WINDING:
                    wantedWinchSpeed = WINCH_SPEED;
                    if((Timer.getFPGATimestamp() - lastWindTime >= WIND_TIMEOUT) || potData.getInches() <= 0){
                        wantedWinchSpeed = 0;
                        shooterState = State.UNWIND_WINCH;
                    }
                    break;
                default:
                    log.logError("Unknown Shooter state: " + shooterState);
                    break;
            }
            if(!IO.USE_PWM_CABLES){
                winchMotor.setX(wantedWinchSpeed);
            }else{
                winchMotorPWM.set(wantedWinchSpeed);
            }
            if(Timer.getFPGATimestamp() - lastLogTime > LOG_EVERY){
                log.logMessage("Current State: " + State.getState(shooterState));
                log.logMessage("Pot Dist: " + potData.getInches());
                log.logMessage("Wanted Winch Speed: " + wantedWinchSpeed);
                lastLogTime = Timer.getFPGATimestamp();
            }
        }
    }
    
    /**
     * Sets the state of the Shooter to State.Shoot if the current state of the robot is standby.
     * @return if the shooter attempted to shoot
     */ 
    public boolean shoot(){
       if(shooterState == State.STANDBY && Acquisitions.getInstance().readyToShoot()){
           shooterState = State.SHOOT;
           log.logMessage("Shooting");
           return true;
       }
       log.logMessage("Attempted to shoot, but could not");
       return false;
    }

    private String subsystemName = "Shooter";
    public void liveWindow() {
        if(!IO.USE_PWM_CABLES){
            LiveWindow.addActuator(subsystemName, "Winch", winchMotor);
        }else{
            LiveWindow.addActuator(subsystemName, "Winch", winchMotorPWM);
        }
        LiveWindow.addSensor(subsystemName, "Winch Pot", winchPot);
        LiveWindow.addActuator(subsystemName, "Fire", latch);
        LiveWindow.addSensor(subsystemName, "Winch Stop Limit", latchSwitch);
        
    }
    
    /**
     * A class used for storing possible states of the Shooter.
     * 
     * SHOOT - Shoots the ball
     * RETRACT - disengages the latch and then sets to WINDIND
     * STANDBY - Does nothing
     * SHOOTER_COOLDOWN - Waits TIME_BETWEEN_SHOTS seconds after the shot, then
     *                    starts winding
     * SET_HOME - Drives the winch all the way back, sets the pot to 0, and then
     *            engages the latch and unwinds the winch
     * UNWIND_WINCH - Engages the latch then sets the state to UNWINDING
     * UNWINDING - Unwinds the winch until the pot reaches INCHES_TO_WIND, then
     *             Goes to Standby
     * WINDING - Winds the winch until it reaches 0 or 
     */ 
    private static class State{
        public static final int SHOOT = 1;
        public static final int RETRACT = 2;
        public static final int STANDBY = 3;
        public static final int SHOOTER_COOLDOWN = 4;
        public static final int SET_HOME = 5;
        public static final int UNWIND_WINCH = 6;
        public static final int UNWINDING = 7;
        public static final int WINDING = 8;
        
        /**
         * Returns a string version of the state.
         */
        public static String getState(int state){
            switch(state){
                case STANDBY:
                    return "Standby";
                case SHOOT:
                    return "Shooting";
                case RETRACT:
                    return "Retracting";
                case SHOOTER_COOLDOWN:
                    return "Shooter is cooling down";
                case SET_HOME:
                    return "Setting home";
                case UNWIND_WINCH:
                    return "Starting Unwinding";
                case UNWINDING:
                    return "Unwinding Winch";
                case WINDING:
                    return "Winding Winch";
            }
            return "UNKOWN MODE: " + state;
        }
    }
}
