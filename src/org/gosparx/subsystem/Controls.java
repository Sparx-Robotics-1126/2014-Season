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
public class Controls extends GenericSubsystem {

    /**
     * The amount in degrees to trim the shooter every time the trim buttons are
     * hit.
     */
    private static final double TRIM_ANGLE = 4;

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
     * Time (in seconds) between offset
     */
    private static final double OFFSET_TIME = 0.5;

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
     * Stores if we are overriding auto shifting
     */
    private boolean shiftingOverride = false;

    /**
     * The last Y value of the left Joystick
     */
    private double lastLeftJoyYValue = 0.0;

    /**
     * The last Y value of the right Joystick
     */
    private double lastRightJoyYValue = 0.0;

    /**
     * The left speed to set the drives
     */
    private double leftSpeedToSet;

    /**
     * The right speed to set the drives
     */
    private double rightSpeedToSet;

    /*
     * The last value of the driver top left button
     */
    private boolean lastDriverLeftTopButton;

    /**
     * The last value of the drivers left trigger
     */
    private boolean lastDriverLeftTrigger;

    /**
     * The last value of the RTWO button.
     */
    private boolean lastShoot = false;

    /**
     * The Time the last shooter angle offset was updated
     */
    private double lastOffsetTime;

    /**
     * The time at which the robot was enabled
     */
    private double startingMatchTime = 0;

    /**
     * The time from last flashes
     */
    private double lastFlashTime;

    /**
     * The time between flashes
     */
    private static final double FLASH_TIME = 0.5;

    /**
     * An instance of acquisitions.
     */
    private Acquisitions acq;

    /**
     * An instance of Shooter for use of accessing non static methods.
     */
    private Shooter shooter;

    /**
     * The last state of the trim up button. Used for rising edge detection.
     */
    private boolean lastTrimUp = false;

    /**
     * The last state of the trim down button. Used for rising edge detection.
     */
    private boolean lastTrimDown = false;

    /**
     * The last state of the circle button. Used for rising edge detection.
     */
    private boolean lastOPCircle = false;

    /**
     * The last state of the cross button. Used for rising edge detection.
     */
    private boolean lastOPCross = false;

    /**
     * The last state of the square button. Used for rising edge detection.
     */
    private boolean lastOPSquare = false;

    /**
     * The last state of the triangle button. Used for rising edge detection.
     */
    private boolean lastOPTriangle = false;

    /**
     * The last state of the R1 button. Used for rising edge detection.
     */
    private boolean lastOPR1 = false;

    /**
     * The last state of the R3 button. Used for rising edge detection.
     */
    private boolean lastOPR3 = false;

    /**
     * The last state of the start button. Used for rising edge detection.
     */
    private boolean lastOPStart = false;

    /**
     * The last value of the operator's select button(PS2) or back button
     * (Logitech)
     */
    private boolean lastOPSelect = false;

    //********************************************************************
    //********************AIRFLO Controller Mapping***********************
    //********************************************************************
    public static final int AIRFLO_CROSS = 1;
    public static final int AIRFLO_CIRCLE = 2;
    public static final int AIRFLO_SQUARE = 3;
    public static final int AIRFLO_TRIANGLE = 4;
    public static final int AIRFLO_LONE = 5;
    public static final int AIRFLO_LTWO = 7;
    public static final int AIRFLO_RONE = 6;
    public static final int AIRFLO_RTWO = 8;
    public static final int AIRFLO_SELECT = 9;
    public static final int AIRFLO_START = 10;
    public static final int AIRFLO_ANALOG = 11;
    public static final int AIRFLO_L3 = 12;
    public static final int AIRFLO_R3 = 13;
    public static final int AIRFLO_RIGHT_Y = 3;
    public static final int AIRFLO_RIGHT_X = 4;
    public static final int AIRFLO_LEFT_Y = 2;
    public static final int AIRFLO_LEFT_X = 1;
    public static final int AIRFLO_D_PAD_Y = 6;//UP - NEGATIVE
    public static final int AIRFLO_D_PAD_X = 5;//LEFT - NEGATIVE 

    //********************************************************************
    //*****************Playstation 2 Controller Mapping*******************
    //********************************************************************
    private static final int LEFT_X_AXIS = 1;
    private static final int LEFT_Y_AXIS = 2;
    private static final int RIGHT_X_AXIS = 4;
    private static final int RIGHT_Y_AXIS = 3;
    /**
     * right == 1, left == -1
     */
    private static final int DPAD_X_AXIS = 5;
    /**
     * down == 1, up == -1
     */
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
    /**
     * right = 1, left = -1
     */
    private static final int LOGI_DPAD_X_AXIS = 5;
    /**
     * up = -1, down = 1
     */
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
    
    private static final int DPAD_DOWN = 1;
    private static final int DPAD_UP = -1;
    private static final int DPAD_LEFT = -1;
    private static final int DPAD_RIGHT = 1;
    

    /**
     * Creates a new Controls
     */
    private Controls() {
        super(Logger.SUB_CONTROLER, Thread.NORM_PRIORITY);
    }

    /**
     * Returns a pointer to the Controls
     *
     * @return the instance to the controls.
     */
    public static Controls getInstance() {
        if (controls == null) {
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
        acq = Acquisitions.getInstance();
        shooter = Shooter.getInstance();
    }

    /**
     * Reassigns all of the variables and sets drives speed to the Y variables
     * of the driver joysticks
     *
     * @throws Exception throws exception if something bad happens
     */
    public void execute() throws Exception {
        if (ds.isEnabled() && ds.isOperatorControl()) {
            lastOPSelect = opSelect;
            lastOPR1 = opR1;
            lastOPR3 = opR3;
            lastOPCircle = opCircle;
            lastOPSquare = opSquare;
            lastOPTriangle = opTriangle;
            lastOPCross = opCross;
            lastOPStart = opStart;
            lastTrimUp = opL1;
            lastTrimDown = opL2;
            lastLeftJoyYValue = driverLeftYAxis;
            lastRightJoyYValue = driverRightYAxis;
            lastShoot = opR2;
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

            /*/****************DRIVER********************* /*/
            if (Math.abs(driverLeftYAxis) < JOYSTICK_DEADZONE) {
                driverLeftYAxis = 0;
            }
            if (Math.abs(driverRightYAxis) < JOYSTICK_DEADZONE) {
                driverRightYAxis = 0;
            }

            drives.forceLowGear((driverRightTrigger));
            if (driverRightTopButton) {
                drives.startHoldPos();
            } else {
                drives.stopHoldPos();
            }
            leftSpeedToSet = getSpeed(driverLeftYAxis, lastLeftJoyYValue);
            rightSpeedToSet = getSpeed(driverRightYAxis, lastRightJoyYValue);
            drives.setSpeed(leftSpeedToSet, rightSpeedToSet);
            if (driverLeftTrigger && driverLeftTopButton && !lastDriverLeftTopButton && !lastDriverLeftTrigger) {
                shiftingOverride = !shiftingOverride;
            } else if (driverLeftTopButton && !driverLeftTrigger && !lastDriverLeftTopButton) {
                drives.manualShiftUp();
            } else if (driverLeftTrigger && !driverLeftTopButton && !lastDriverLeftTrigger) {
                drives.manualShiftDown();
            }
            drives.setManualShifting(shiftingOverride);
            /*/********************OPERATOR****************** /*/
            if (opCircle && !lastOPCircle) {
                acq.setMode(Acquisitions.AcqState.ACQUIRING);
            } else if (opCross && !lastOPCross) {
                acq.setMode(Acquisitions.AcqState.OFF_STATE);
            } else if (opTriangle && !lastOPTriangle) {
                acq.setMode(Acquisitions.AcqState.EJECT_BALL);
            } else if (opStart && !lastOPStart) {
                acq.setPreset(Acquisitions.AcqState.TRUSS_SHOT_PRESET);
                shooter.setAdjustSlack(3.50);
            } else if (opSquare && !lastOPSquare) {
                acq.setMode(Acquisitions.AcqState.SAFE_STATE);
            }

            if (opDPadYAxis == DPAD_DOWN) {
                acq.setPreset(Acquisitions.AcqState.FAR_SHOOTER_PRESET);
                shooter.setAdjustSlack(Shooter.MAX_UNWIND_INCHES);
            } else if (opDPadXAxis == DPAD_RIGHT) {
                acq.setPreset(Acquisitions.AcqState.MIDDLE_SHOOTER_PRESET);
                shooter.setAdjustSlack(Shooter.MAX_UNWIND_INCHES);
            } else if (opDPadYAxis == DPAD_UP) {
                acq.setPreset(Acquisitions.AcqState.ONE_POINT_PRESET);
                shooter.setAdjustSlack(Shooter.MIN_UNWIND_INCHES + 0.85);
            } else if (opDPadXAxis == DPAD_LEFT) {
                acq.setPreset(Acquisitions.AcqState.TRUSS_SHOT_PRESET);
                shooter.setAdjustSlack(2.75);
            }
            if(opLeftYAxis == 1){
                shooter.setAdjustSlack(Shooter.MIN_UNWIND_INCHES);
            }else if(opLeftYAxis == -1){
                shooter.setAdjustSlack(Shooter.MAX_UNWIND_INCHES);
            }
            //OFFSET
            if (Timer.getFPGATimestamp() - OFFSET_TIME >= lastOffsetTime && ds.isEnabled() && (opL1 || opL2)) {
                lastOffsetTime = Timer.getFPGATimestamp();
                if (opL2 && !lastTrimDown) {
                    acq.addOffset(TRIM_ANGLE);
                } else if (opL1 && !lastTrimUp) {
                    acq.addOffset(-TRIM_ANGLE);
                }
            }

            if (opR1 && !lastOPR1) {
                shooter.setMode(Shooter.State.SET_HOME);
            } else if (opR3 && !lastOPR3) {
                shooter.setMode(Shooter.State.STANDBY);
            } else if (opL3) {
                shooter.setMode(Shooter.State.SHOOTER_WINDING);
            }

            if (opR2 && !lastShoot) {
                shooter.shoot();
            }
            
            if(opSelect && !lastOPSelect){
                acq.setBrakeEnabled(!acq.isBrakeEnabled());
            }
            
            if(opRightYAxis > 0.5){
                acq.setManaulAcq(true);
            }else{
                acq.setManaulAcq(false);
            }
            
            smartDashboardTimer();
        } else {
            startingMatchTime = Timer.getFPGATimestamp();
        }
    }

    /**
     * Edit this method to change how the joystick values translate into speed
     *
     * @param joystickValue - the joystick value to convert to speed
     * @param lastValue - the last value of the joystick
     * @return the speed desired after the joystickValue is applied to the
     * formula
     */
    private double getSpeed(double joystickValue, double lastValue) {
        if (Math.abs(joystickValue) < JOYSTICK_DEADZONE) {
            joystickValue = 0;
        }
        return -joystickValue;
    }

    public void liveWindow() {
        SmartDashboard.putNumber("Timer", 0);
        SmartDashboard.putBoolean("10 Seconds Left", false);
    }

    private void smartDashboardTimer() {
        SmartDashboard.putNumber("Timer", Timer.getFPGATimestamp() - startingMatchTime);
        if (Timer.getFPGATimestamp() - startingMatchTime > 130 && Timer.getFPGATimestamp() - startingMatchTime < 140) {//130, 140
            if (Timer.getFPGATimestamp() - lastFlashTime >= FLASH_TIME) {
                SmartDashboard.putBoolean("10 Seconds Left", !SmartDashboard.getBoolean("10 Seconds Left"));
                lastFlashTime = Timer.getFPGATimestamp();
            }
        }
    }

    public int sleepTime() {
        return 20;
    }

    public void logInfo() {
        log.logMessage("Left Speed to Set: " + leftSpeedToSet + " Right Speed to Set: " + rightSpeedToSet);
        log.logMessage("Right Joystick Y: " + driverRightYAxis + " Right Joystick Last Y: " + lastRightJoyYValue);
        log.logMessage("Average Runtime: " + getAverageRuntime() + " sec");
    }
}
