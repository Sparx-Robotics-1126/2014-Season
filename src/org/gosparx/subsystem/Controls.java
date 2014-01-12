/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.Joystick;

/**
 * @author Alex
 * @date 1/12/14
 */
public class Controls extends GenericSubsystem{
    
    private static Controls controls;
    
    private final int LEFT_DRIVER_JOY_PORT                                  = 1;
    
    private final int RIGHT_DRIVER_JOY_PORT                                 = 2;
    
    private final int OPER_JOY_PORT                                         = 3;
    
    private Joystick leftJoy;
    
    private Joystick rightJoy;
    
    private Joystick opJoy;
    
    
    //********************************************************************
    //*****************Playstation 2 Controller Mapping*******************
    //********************************************************************
    //TODO Check these values
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
    //*******************Attack 3 Controller Mapping**********************
    //********************************************************************
    //TODO Check these values
    private final int ATTACK3_Y_AXIS = 2;
    private final int ATTACK3_X_AXIS = 2;
    private final int ATTACK3_Z_AXIS = 3;
    private final int ATTACK3_TRIGGER = 1;    
    private final int ATTACK3_TOP_BUTTON = 2;
    
    //********************************************************************
    //**********************Operator Joy Vars*****************************
    //********************************************************************
    private int opLeftXAxis;
    private int opLeftYAxis;
    private int opRightXAxis;
    private int opRightYAxis;
    private int opDPadXAxis;
    private int opDPadYAxis;
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
    
    private Controls(){
        super("Controls", Thread.MAX_PRIORITY);
    }
    
    public static Controls getInstance(){
        if(controls == null){
            controls = new Controls();
        }
        return controls;
    }
    
    public void init() {
        leftJoy = new Joystick(LEFT_DRIVER_JOY_PORT);
        rightJoy = new Joystick(RIGHT_DRIVER_JOY_PORT);
        opJoy = new Joystick(OPER_JOY_PORT);
    }

    public void execute() throws Exception {
        while(true){
            
        }
    }
    
    //TODO Rename all of these to their mapped functions;
    public int getOpLeftX(){
        return opLeftXAxis;
    }
    
    public int getOpLeftY(){
        return opLeftYAxis;
    }
    
    public int getOpRightX(){
        return opRightXAxis;
    }
    
    public int getOpRightY(){
        return opRightYAxis;
    }
    
}
