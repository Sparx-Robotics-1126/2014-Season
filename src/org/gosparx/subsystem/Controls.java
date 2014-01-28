/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.gosparx.subsystem;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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
     * The dead zone for the Driver joysticks. This is the zone in which the    
     * drives will be set to 0.
     */ 
    private double JOYSTICK_DEADZONE = .04; 
    
    /**
     * The last time the desired speed was logged
     */
    private double lastLogTime = 0;
    
    /**
     * Time in seconds between logging the desired speeds
     */
    private double LOG_EVERY = 5.0;
    
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
     * The last state of the hold in place start button
     */ 
    private boolean lastHoldInPlaceStart                                = false;
    
    /**
     * The last state of the hold in place stop button
     */    
    private boolean lastHoldInPlaceStop                                 = false;
    
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
        while(true){
            if(ds.isEnabled() && ds.isOperatorControl()){
                lastShiftDown = driverLeftTrigger;
                lastShiftUp = driverRightTrigger;
                lastShiftOverrideState = driverLeftTopButton;
                lastHoldInPlaceStart = opStart;
                lastHoldInPlaceStop = opSelect;
                opLeftXAxis = opJoy.getRawAxis(LEFT_X_AXIS);
                opLeftYAxis = opJoy.getRawAxis(LEFT_Y_AXIS);
                opRightXAxis = opJoy.getRawAxis(RIGHT_X_AXIS);
                opRightYAxis = opJoy.getRawAxis(RIGHT_Y_AXIS);
                opDPadXAxis = opJoy.getRawAxis(DPAD_X_AXIS);
                opDPadYAxis = opJoy.getRawAxis(DPAD_Y_AXIS);
                opTriangle = opJoy.getRawButton(TRIANGLE);
                opCircle = opJoy.getRawButton(CIRCLE);
                opSquare = opJoy.getRawButton(SQUARE);
                opCross = opJoy.getRawButton(CROSS);
                opStart = opJoy.getRawButton(START);
                opSelect = opJoy.getRawButton(SELECT);
                opL1 = opJoy.getRawButton(LONE);
                opL2 = opJoy.getRawButton(LTWO);
                opL3 = opJoy.getRawButton(L3);
                opR1 = opJoy.getRawButton(RONE);
                opR2 = opJoy.getRawButton(RTWO);
                opR3 = opJoy.getRawButton(R3);
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

                if(driverLeftTopButton && !lastShiftOverrideState){
                    shiftingOverride = !shiftingOverride;
                    log.logMessage("Toggled manual shifting");
                }
                if(driverLeftTrigger && !shiftingOverride){
                    drives.forceLowGear(true);
                }else{
                    drives.forceLowGear(false);
                }
                if(shiftingOverride){
                    if(driverLeftTrigger && !lastShiftDown){
                        drives.manualShiftDown();
                    }else if(driverRightTrigger && !lastShiftUp){
                        drives.manualShiftUp();
                    }
                }
                if(!lastHoldInPlaceStart && opStart){
                    drives.startHoldPos();
                }else if(!lastHoldInPlaceStop && opSelect){
                    drives.stopHoldPos();
                }
                drives.setManualShifting(shiftingOverride);
                
                drives.setSpeed(Drives.MAX_ROBOT_SPEED * ((driverLeftYAxis > 0) ? MathUtils.pow(driverLeftYAxis,2): -MathUtils.pow(driverLeftYAxis,2)) * -1, Drives.MAX_ROBOT_SPEED * ((driverRightYAxis > 0) ? MathUtils.pow(driverRightYAxis,2): -MathUtils.pow(driverRightYAxis,2)) * -1);
                
                if(Timer.getFPGATimestamp() - LOG_EVERY >= lastLogTime){
                    lastLogTime = Timer.getFPGATimestamp();
                    log.logMessage("Left: " + Drives.MAX_ROBOT_SPEED * driverLeftYAxis * -1 + " Right: " + Drives.MAX_ROBOT_SPEED * driverRightYAxis * -1);
                }
                Thread.sleep(20);
            }
        }
    }
}