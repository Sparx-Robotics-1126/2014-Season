/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.Joystick;
import org.gosparx.IO;

/**
 * @author Alex
 * @date 1/12/14
 */
public class Controls extends GenericSubsystem{
    
    private static Controls controls;
    
    private Joystick leftJoy;
    
    private Joystick rightJoy;
    
    private Joystick opJoy;
    
    private Drives drives;
    
    
    //********************************************************************
    //*****************Playstation 2 Controller Mapping*******************
    //********************************************************************
    private final int LEFT_X_AXIS = 1;
    private final int LEFT_Y_AXIS = 2;
    private final int RIGHT_X_AXIS = 4;
    private final int RIGHT_Y_AXIS = 3;
    private final int DPAD_X_AXIS = 5;// right == 1, left == -1
    private final int DPAD_Y_AXIS = 6;// down == 1, up == -1
    private final int TRIANGLE = 1;
    private final int CIRCLE = 2;
    private final int CROSS = 3;
    private final int SQUARE = 4;
    private final int LTWO = 5;
    private final int RTWO = 6;
    private final int LONE = 7;
    private final int RONE = 8;
    private final int SELECT = 9;
    private final int START = 10;
    private final int L3 = 11;
    private final int R3 = 12;
    
    //********************************************************************
    //*******************Driver Controller Mapping**********************
    //********************************************************************
    private final int ATTACK3_Y_AXIS = 2;
    private final int ATTACK3_X_AXIS = 2;
    private final int ATTACK3_Z_AXIS = 3;
    private final int ATTACK3_TRIGGER = 1;    
    private final int ATTACK3_TOP_BUTTON = 2;
    
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
        super("Controls", Thread.NORM_PRIORITY);
    }
    /**
     * Returns a pointer to the Controls 
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
     */
    public void execute() throws Exception {
        while(true){
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
            drives.setSpeed(driverLeftYAxis, driverRightYAxis);
            Thread.sleep(20);
        }
    }
}