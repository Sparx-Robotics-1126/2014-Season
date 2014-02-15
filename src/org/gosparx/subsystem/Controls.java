package org.gosparx.subsystem;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.gosparx.IO;
import org.gosparx.util.Logger;

/**
 * @author Alex
 * @date 1/12/14
 */
public class Controls extends GenericSubsystem{
    
    /**
     * The only Controls that will ever be created in the entire robot. It is 
     * returned by getInstance()
     */
    private static Controls controls;
    
    /**
     * The left driver joystick. Its port is stored in IO.java
     */
    private Joystick leftJoy;
    
    /**
     * The right driver joystick. Its port is stored in IO.java
     */
    private Joystick rightJoy;
    
    /**
     * The operator joystick. Its port is stored in IO.java
     */
    private Joystick opJoy;
    
    /**
     * An instance of Drives
     */ 
    private Drives drives;
    
    /**
     * The factor to divide the previous + the current joystick Y values. Used 
     * to slow down the drives to a complete stop so that they do not flip the 
     * robot on a sudden stop
     */ 
    private static final double SLOW_DOWN_RAMP = 1.35;
    
    /**
     * The dead zone for the Driver joysticks. This is the zone in which the    
     * drives will be set to 0.
     */ 
    private double JOYSTICK_DEADZONE = .04; 
        
    /**
     * Stores if we are overriding auto shifting
     */
    private boolean shiftingOverride                                    = false;
    
    /**
     * The last state of the shifting override button
     */
    private boolean lastShiftOverrideState                              = false;
    
    /**
     * The last state of the manual shift up button
     */ 
    private boolean lastShiftUp                                         = false;
    
    /**
     * The last state of the manual shift down
     */ 
    private boolean lastShiftDown                                       = false;
    
    /**
     * The last state of the hold in place button
     */
    private boolean lastHoldInPlace                                     = false;
    
    /**
     * The last Y value of the left Joystick
     */ 
    private double lastLeftJoyYValue                                      = 0.0;
    
    /**
     * The last Y value of the right Joystick
     */ 
    private double lastRightJoyYValue                                     = 0.0;
    
    /**
     * The left speed to set the drives
     */ 
    private double leftSpeedToSet;
    
    /**
     * The right speed to set the drives
     */ 
    private double rightSpeedToSet;
    
    //********************************************************************
    //*****************Playstation 2 Controller Mapping*******************
    //********************************************************************
    private static final int LEFT_X_AXIS = 1;
    private static final int LEFT_Y_AXIS = 2;
    private static final int RIGHT_X_AXIS = 4;
    private static final int RIGHT_Y_AXIS = 3;
    /** right == 1, left == -1 */
    private static final int DPAD_X_AXIS = 5;
    /** down == 1, up == -1 */
    private static final int DPAD_Y_AXIS = 6;
    private static final int TRIANGLE = 1;
    private static final int CIRCLE = 2;
    private static final int CROSS = 3;
    private static final int SQUARE = 4;
    private static final int LTWO = 5;
    private static final int RTWO = 6;
    private static final int LONE = 7;
    private static final int RONE = 8;
    private static final int SELECT = 9;
    private static final int START = 10;
    private static final int L3 = 11;
    private static final int R3 = 12;
    
    //**************************************************************************
    //*****************************Logitech f310 mapping************************
    //**************************************************************************
    private static final int LOGI_LEFT_X_AXIS = 1;
    private static final int LOGI_LEFT_Y_AXIS = 2;
    private static final int LOGI_RIGHT_X_AXIS = 3;
    private static final int LOGI_RIGHT_Y_AXIS = 4;
    /** right = 1, left = -1 */
    private static final int LOGI_DPAD_X_AXIS = 5;
    /** up = -1, down = 1 */
    private static final int LOGI_DPAD_Y_AXIS = 6;
    private static final int LOGI_X = 1;
    private static final int LOGI_A = 2;
    private static final int LOGI_B = 3;
    private static final int LOGI_Y = 4;
    private static final int LOGI_L1 = 5;
    private static final int LOGI_R1 = 6;
    private static final int LOGI_L2 = 7;
    private static final int LOGI_R2 = 8;
    private static final int LOGI_BACK = 9;
    private static final int LOGI_START = 10;
    private static final int LOGI_L3 = 11;
    private static final int LOGI_R3 = 12;
    
    //********************************************************************
    //*******************Driver Controller Mapping**********************
    //********************************************************************
    private static final int ATTACK3_Y_AXIS = 2;
    private static final int ATTACK3_X_AXIS = 2;
    private static final int ATTACK3_Z_AXIS = 3;
    private static final int ATTACK3_TRIGGER = 1;    
    private static final int ATTACK3_TOP_BUTTON = 2;
    
    //********************************************************************
    //**********************Operator Joy Vars*****************************
    //********************************************************************
    private double opLeftXAxis;
    private double opLeftYAxis;
    private double opRightXAxis;
    private double opRightYAxis;
    private double opDPadXAxis;
    private double opDPadYAxis;
    private boolean opTriangle;
    private boolean opCircle;
    private boolean opCross;
    private boolean opSquare;
    private boolean opL2;
    private boolean opR2;
    private boolean opL1;
    private boolean opR1;
    private boolean opStart;
    private boolean opSelect;
    private boolean opL3;
    private boolean opR3;
    //********************************************************************
    //***********************Driver Joy Vars******************************
    //********************************************************************
    private double driverLeftYAxis;
    private double driverLeftXAxis;
    private double driverLeftZAxis;
    private boolean driverLeftTrigger;
    private boolean driverLeftTopButton;
    private double driverRightYAxis;
    private double driverRightXAxis;
    private double driverRightZAxis;
    private boolean driverRightTrigger;
    private boolean driverRightTopButton;
    private boolean lastDriverRightTrigger;
    private boolean lastDriverLeftTopButton;
    private boolean lastDriverLeftTrigger;
    /**
     * Creates a new Controls
     */
    private Controls(){
        super(Logger.SUB_CONTROLER, Thread.NORM_PRIORITY);
    }
    /**
     * Returns a pointer to the Controls 
     * 
     * @return the instance to the controls.
     */
    public static Controls getInstance(){
        if(controls == null){
            controls = new Controls();
        }
        return controls;
    }
    /**
     * Creates the joystick objects and grabs an instance of the Drives 
     * subsystem 
     */
    public void init() {
        leftJoy = new Joystick(IO.LEFT_DRIVER_JOY_PORT);
        rightJoy = new Joystick(IO.RIGHT_DRIVER_JOY_PORT);
        opJoy = new Joystick(IO.OPER_JOY_PORT);
        drives = Drives.getInstance();
    }
    /**
     * Reassigns all of the variables and sets drives speed to the Y variables 
     * of the driver joysticks
     * 
     * @throws Exception throws exception if something bad happens
     */
    public void execute() throws Exception {
        while(!ds.isTest()){
            if(ds.isEnabled() && ds.isOperatorControl()){                
                lastLeftJoyYValue = driverLeftYAxis;
                lastRightJoyYValue = driverRightYAxis;
                lastShiftDown = driverLeftTrigger;
                lastShiftUp = driverRightTrigger;
                lastShiftOverrideState = driverLeftTopButton;
                lastHoldInPlaceStart = driverRightTopButton;
                lastDriverRightTrigger = driverRightTrigger;
                lastDriverLeftTopButton = driverLeftTopButton;
                lastDriverLeftTrigger = driverLeftTrigger;
                opLeftXAxis = opJoy.getRawAxis(LOGI_LEFT_X_AXIS);
                opLeftYAxis = opJoy.getRawAxis(LOGI_LEFT_Y_AXIS);
                opRightXAxis = opJoy.getRawAxis(LOGI_RIGHT_X_AXIS);
                opRightYAxis = opJoy.getRawAxis(LOGI_RIGHT_Y_AXIS);
                opDPadXAxis = opJoy.getRawAxis(LOGI_DPAD_X_AXIS);
                opDPadYAxis = opJoy.getRawAxis(LOGI_DPAD_Y_AXIS);
                opTriangle = opJoy.getRawButton(LOGI_Y);
                opCircle = opJoy.getRawButton(LOGI_B);
                opSquare = opJoy.getRawButton(LOGI_X);
                opCross = opJoy.getRawButton(LOGI_A);
                opStart = opJoy.getRawButton(LOGI_START);
                opSelect = opJoy.getRawButton(LOGI_BACK);
                opL1 = opJoy.getRawButton(LOGI_L1);
                opL2 = opJoy.getRawButton(LOGI_L2);
                opL3 = opJoy.getRawButton(LOGI_L3);
                opR1 = opJoy.getRawButton(LOGI_R1);
                opR2 = opJoy.getRawButton(LOGI_R2);
                opR3 = opJoy.getRawButton(LOGI_R3);
                driverLeftXAxis = leftJoy.getRawAxis(ATTACK3_X_AXIS);
                driverLeftYAxis = leftJoy.getRawAxis(ATTACK3_Y_AXIS);
                driverLeftZAxis = leftJoy.getRawAxis(ATTACK3_Z_AXIS);
                driverLeftTopButton = leftJoy.getRawButton(ATTACK3_TOP_BUTTON);
                driverLeftTrigger = leftJoy.getRawButton(ATTACK3_TRIGGER);
                driverRightXAxis = rightJoy.getRawAxis(ATTACK3_X_AXIS);
                driverRightYAxis = rightJoy.getRawAxis(ATTACK3_Y_AXIS);
                driverRightZAxis = rightJoy.getRawAxis(ATTACK3_Z_AXIS);
                driverRightTopButton = rightJoy.getRawButton(ATTACK3_TOP_BUTTON);
                driverRightTrigger = rightJoy.getRawButton(ATTACK3_TRIGGER);
                
                if(Math.abs(driverLeftYAxis) < JOYSTICK_DEADZONE){
                    driverLeftYAxis = 0;
                }
                if(Math.abs(driverRightYAxis) < JOYSTICK_DEADZONE){
                    driverRightYAxis = 0;
                }
                
                drives.forceLowGear((driverRightTrigger&&lastDriverRightTrigger));
                if(lastHoldInPlaceStart && driverRightTopButton){
                    drives.startHoldPos();
                    }else{
                        drives.stopHoldPos();
                    }
                }
                leftSpeedToSet = getSpeed(driverLeftYAxis, lastLeftJoyYValue);
                rightSpeedToSet = getSpeed(driverRightYAxis, lastRightJoyYValue);
                drives.setSpeed(leftSpeedToSet, rightSpeedToSet);
                }else{
                    drives.stopHoldPos();
                }
                if(driverLeftTrigger && driverLeftTopButton){
                    shiftingOverride = !shiftingOverride;
                }else if(driverLeftTopButton && !driverLeftTrigger && !lastDriverLeftTopButton){
                    drives.manualShiftUp();
                }else if(driverLeftTrigger && !driverLeftTopButton && !lastDriverLeftTrigger){
                    drives.manualShiftDown();
                drives.setManualShifting(shiftingOverride);
                
                if(Timer.getFPGATimestamp() - LOG_EVERY >= lastLogTime){
                    lastLogTime = Timer.getFPGATimestamp();
                    log.logMessage("Left Speed to Set: " + leftSpeedToSet + " Right Speed to Set: " + rightSpeedToSet);
                    log.logMessage("Left Joystick Y: " + driverLeftYAxis + " Left Joystick Last Y: " + lastLeftJoyYValue);
                    log.logMessage("Right Joystick Y: " + driverRightYAxis + " Right Joystick Last Y: " + lastRightJoyYValue);
                }
                Thread.sleep(20);
            }
        }
    }
    /**
     * Edit this method to change how the joystick values translate into speed
     * @param joystickValue - the joystick value to convert to speed
     * @param lastValue - the last value of the joystick
     * @return the speed desired after the joystickValue is applied to the formula
     */
    private double getSpeed(double joystickValue, double lastValue){
        if(joystickValue > lastValue){
            joystickValue = (joystickValue + lastValue)/SLOW_DOWN_RAMP;//closer to 1 = slower   
        }
        if(Math.abs(joystickValue) < JOYSTICK_DEADZONE){
            joystickValue = 0;
        }
        return (Drives.MAX_ROBOT_SPEED * ((joystickValue > 0) ? MathUtils.pow(joystickValue,2): -MathUtils.pow(joystickValue,2)) * -1);
    }

    public void liveWindow() {
        
    }
    
    private void smartDashboardTimer(){
        SmartDashboard.putNumber("Timer", ds.getMatchTime());
    }
}