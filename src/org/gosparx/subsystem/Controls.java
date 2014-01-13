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
            opLeftXAxis = opJoy.getRawAxis(LEFT_X_AXIS);
            opLeftYAxis = opJoy.getRawAxis(LEFT_Y_AXIS);
            opRightXAxis = opJoy.getRawAxis(RIGHT_X_AXIS);
            opRightYAxis = opJoy.getRawAxis(RIGHT_Y_AXIS);
        }
    }
    
    //TODO Rename all of these to their mapped functions;
    public double getOpLeftX(){
        return opLeftXAxis;
    }
    
    public double getOpLeftY(){
        return opLeftYAxis;
    }
    
    public double getOpRightX(){
        return opRightXAxis;
    }
    
    public double getOpRightY(){
        return opRightYAxis;
    }
    
    public double getOpDPadX(){
        return opDPadYAxis;
    }
    
    public double getOpDPadY(){
        return opDPadYAxis;
    }
    
    public boolean getOpTriangle(){
        return opTriangle;
    }
    
    public boolean getOpCross(){
        return opCross;
    }
    
    public boolean getOpCircle(){
        return opCircle;
    }
    
    public boolean getOpSquare(){
        return opSquare;
    }
    
    public boolean getOpSelect(){
        return opSelect;
    }
    
    public boolean getOpStart(){
        return opStart;
    }
    
    public boolean getOpL1(){
        return opL1;
    }
    
    public boolean getOpL2(){
        return opL2;
    }
    
    public boolean getOpL3(){
        return opL3;
    }
    
    public boolean getOpR1(){
        return opR1;
    }
    
    public boolean getOpR2(){
        return opR2;
    }
    
    public boolean getOpR3(){
        return opR3;
    }
    
    public double getDriverLeftY(){
        return driverLeftYAxis;
    }
    
    public double getDriverLeftX(){
        return driverLeftXAxis;
    }
    
    public double getDriverLeftZ(){
        return driverLeftZAxis;
    }
    
    public boolean getDriverLeftTrigger(){
        return driverLeftTrigger;
    }
    
    public boolean getDriverLeftTopButton(){
        return driverLeftTopButton;
    }
    
    public double getDriverRightY(){
        return driverRightYAxis;
    }
    
    public double getDriverRightX(){
        return driverRightXAxis;
    }
    
    public double getDriverRightZ(){
        return driverRightZAxis;
    }
    
    public boolean getDriverRightTrigger(){
        return driverRightTrigger;
    }
    
    public boolean getDriverRightTopButton(){
        return driverRightTopButton;
    }
}
