package org.gosparx;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.gosparx.subsystem.Acquisitions;
import org.gosparx.subsystem.Drives;
import org.gosparx.subsystem.GenericSubsystem;
import org.gosparx.subsystem.Shooter;
import org.gosparx.subsystem.Vision;
import org.gosparx.util.Logger;


public class Autonomous extends GenericSubsystem{
    /**
     * Auto instance
     */
    private static Autonomous auto;
    
    /**
     * Instance of drives
     */
    private Drives drives;
    
    /**
     * Instance of vision
     */
    private Vision vision;
    
    /**
     * Instance of acquisitions
     */
    private Acquisitions acq;
    
    /**
     * Instance of shooter
     */
    private Shooter shooter;
    
    /**
     * A list of choices for Smart autonomous mode
     */
    private SendableChooser smartChoose;
    
    /**
     * The autonomous that runAuto uses
     */
    private int[][] currentAutonomous;
    
    /**
     * Test to see if it is "ok" to run the runAuto method
     */
    private boolean runAutonomous = false;
    
    /**
     * Uses for the loop array option
     */
    private int loopTime = 0;
    
    /**
     * Distance from camera to target (closest one)
     */
    private double visionDistance = 0.0;
    
    /**
     * The angle from the camera to the target (closest one)
     */
    private double visionAngle = 0.0;
    
    /**
     * Test to see if the closest target is the hot goal
     */
    private boolean visionHotGoal = false;
    
    /**
     * This is the number of the command that is running
     */
    private int i = 0;

    /**
     * If true than smartDashboard decides the auto mode.
     * Is set by the smart dashboard
     */
    private boolean smartAutoMode = false;
    
    /**
     * The name of the currently selected auto mode
     */
    private String selectedAutoName = "UNKNOWN";
    
    /**
     * Wanted auto mode for autonomous
     */
    private int wantedAutoMode;
    
    /**
     * The auto switch on the robot. It tells what auto to run
     */
    private AnalogChannel autoSelectSwitch;
    
    /**    
     * String used in sendSmartAuto(), used as the group for the current auto name
     * on the livewindow
     */ 
    private String smartChooseName = "Current Auto";
    
    /**
     * The String used for the group of the boolean data that we use to decide 
     * if we are using smartdashboard to choose an automode.
     */ 
    private String smartChooser = "Use SmartDashboard";
    /**************************************************************************/
    /*************************Manual Switch Voltages **************************/
    /**************************************************************************/
    private static final double AUTO_SETTING_0 = 5.00;
    private static final double AUTO_SETTING_1 = 4.45;
    private static final double AUTO_SETTING_2 = 3.89;
    private static final double AUTO_SETTING_3 = 3.33;
    private static final double AUTO_SETTING_4 = 2.77;
    private static final double AUTO_SETTING_5 = 2.22;
    private static final double AUTO_SETTING_6 = 1.66;
    private static final double AUTO_SETTING_7 = 1.10;
    private static final double AUTO_SETTING_8 = 0.54;
    private static final double AUTO_SETTING_9 = -0.1;
    /**************************************************************************/
    /************************ Autonomous commands *****************************/
    /**************************************************************************/
    
    /* Drives */
    private static final int DRIVES_GO_FORWARD              = 1;//distance(inches), speed(double)
    private static final int DRIVES_GO_REVERSE              = 2;//distance(inches), speed(double)
    private static final int DRIVES_TURN_RIGHT              = 3;//Degrees(0 - infinity)
    private static final int DRIVES_TURN_LEFT               = 4;//Degrees(0 - infinity)
    private static final int DRIVES_STOP                    = 5;
    private static final int DRIVES_DONE                    = 6;
    
    /* Aquire */
    private static final int ACQ_READY                      = 10;
    private static final int ACQ_AQUIRE_BALL                = 11;
    private static final int ACQ_REVERSE                    = 12;
    private static final int ACQ_ACQUIRE_IN_POSITION        = 13;
    private static final int ACQ_DONE                       = 14;
    
    /* Shooter */
    private static final int SHOOTER_SHOOT                  = 20;
    private static final int SHOOTER_SET_PRESET             = 21;
    private static final int SHOOTER_READY_TO_SHOOT         = 22;
    private static final int SHOOTER_READY                   = 23;
    
    /* Vision */
    private static final int VISION_DISTANCE                = 30;
    private static final int VISION_ANGLE                   = 31;
    private static final int VISION_HOT_TARGET              = 32;
    
    private static final int DRIVES_TRACK_TARGET            = 40;

    /* Misc */
    private static final int NEXT                           = 96;//next i, how many lines up to repeat
    private static final int LOOP                           = 97;//number of loops, 
    private static final int WAIT                           = 98;
    private static final int END                            = 99;
    /**************************************************************************/
    /**************************** End of commands *****************************/
    /**************************************************************************/
    
    /**
     * No auto will run
     */
    private static final String NO_AUTO_NAME = "No Auto";
    private static final int[][] noAuto = { 
        {END}
    };
    
    /**
     * Drives forward 20 feet
     */
    private static final String MOVE_FOWARD_NAME = "Move Foward";
    private static final int[][] moveFoward = {
        {DRIVES_GO_FORWARD, 20*12},  
        {DRIVES_DONE},
        {END}
    };
    
    /**
     * Drives in a 4x4 foot square, turning to the right
     */
    private static final String AUTO_SQUARE_NAME = "Auto Square";
    private static final int[][] autoSquare = {
        {DRIVES_DONE},
        {LOOP, 4},
        {DRIVES_GO_FORWARD, 12},
        {DRIVES_DONE},
        {DRIVES_TURN_RIGHT, 90},
        {DRIVES_DONE},
        {NEXT, 4},
        {END}
    };
    
    /**
     * Camera will follow the target
     */
    private static final String CAMERA_FOLLOW_NAME = "Camera Follow";
    private static final int[][] cameraFollow = { 
        {LOOP, Integer.MAX_VALUE},
        {VISION_DISTANCE},
        {VISION_ANGLE},
        {DRIVES_TRACK_TARGET},
        {NEXT, 3},
        {END}
    };
    
    /**
     * Turns 90 degrees to the left. Used for debugging
     */
    private static final String TURN_90_NAME = "Turn 90";
    private static final int[][] turn90 = {
        {DRIVES_DONE},
        {DRIVES_TURN_LEFT, 90},
        {DRIVES_DONE},
        {END}
    };
   
    
    private static final String TWO_BALLS_IN_HIGH = "Two balls in high";
    private static final int[][] twoBallsInHigh = {
        {ACQ_READY},
        {SHOOTER_SET_PRESET, Acquisitions.AcqState.MIDDLE_SHOOTER_PRESET},
        {SHOOTER_READY_TO_SHOOT},
        {WAIT, 1000},
        {ACQ_AQUIRE_BALL},
        {ACQ_ACQUIRE_IN_POSITION},
        {DRIVES_GO_FORWARD, 30},
        {DRIVES_DONE},
        {SHOOTER_SET_PRESET, Acquisitions.AcqState.MIDDLE_SHOOTER_PRESET},
        {SHOOTER_READY_TO_SHOOT},
        {END}
    };
    
    private static final String ONE_BALL_IN_HIGH = "One ball in high";
    private static final int[][] oneBallInHigh = {
        {ACQ_READY},
        {SHOOTER_READY},
        {SHOOTER_SET_PRESET, Acquisitions.AcqState.FAR_SHOOTER_PRESET},
        {SHOOTER_READY_TO_SHOOT},
        {SHOOTER_SHOOT},
        {END}
    };
    
    /**
     * Autonomous Constructor
     */
    private Autonomous(){
        super(Logger.SUB_AUTONOMOUS, GenericSubsystem.NORM_PRIORITY);   
    }
    
    /**
     * @return the Autonomous class 
     */
    public static Autonomous getInstance(){
        if(auto == null){
            auto = new Autonomous();
        }
        return auto;
    }
    
    /**
     * Gets the current auto mode based off of the auto switch
     */
    public void getAutoMode(){
        
        if(smartAutoMode){
            wantedAutoMode = ((Integer) smartChoose.getSelected()).intValue();
        }else{
           double voltage = autoSelectSwitch.getVoltage(); // need voltage reaading
           if (voltage >= AUTO_SETTING_0){
               wantedAutoMode = 0;
           }else if (voltage >= AUTO_SETTING_1){
               wantedAutoMode = 1;
           }else if (voltage >= AUTO_SETTING_2){
               wantedAutoMode = 2;
           }else if (voltage >= AUTO_SETTING_3){
               wantedAutoMode = 3;
           }else if (voltage >= AUTO_SETTING_4){
               wantedAutoMode = 4;
           }else if (voltage >= AUTO_SETTING_5){
               wantedAutoMode = 5;
           }else if (voltage >= AUTO_SETTING_6){
               wantedAutoMode = 6;
           }else if (voltage >= AUTO_SETTING_7){
               wantedAutoMode = 7;
           }else if (voltage >= AUTO_SETTING_8){
               wantedAutoMode = 8;
           }else if (voltage >= AUTO_SETTING_9){
               wantedAutoMode = 9;
           }else{
               wantedAutoMode = 100;
           }
        }
        switch(wantedAutoMode){
            case 0:
                currentAutonomous = noAuto;
                selectedAutoName = NO_AUTO_NAME;
                break;
            case 1:
                currentAutonomous = autoSquare;
                selectedAutoName = AUTO_SQUARE_NAME;
                break;
            case 2:
                currentAutonomous = cameraFollow;
                selectedAutoName = CAMERA_FOLLOW_NAME;
                break;
            case 3:
                currentAutonomous = moveFoward;
                selectedAutoName = MOVE_FOWARD_NAME;
                break;
            case 4:
                currentAutonomous = turn90;
                selectedAutoName = TURN_90_NAME;
                break;
            case 5:
                currentAutonomous = twoBallsInHigh;
                selectedAutoName = TWO_BALLS_IN_HIGH;
                break;
            case 6:
                currentAutonomous = oneBallInHigh;
                selectedAutoName = ONE_BALL_IN_HIGH;
                break;
            case 7:
                
                break;
            case 8:
                
                break;
            case 9:
                
                break;
            default:
                currentAutonomous = noAuto;
                selectedAutoName = "ERROR";
        }
        sendSmartAuto(selectedAutoName);
    }
    
    
    /**
     * Gets the data from the array and tells each subsystem what actions to take.
     */
    private void runAutonomous(){
        int start = 0, finished = currentAutonomous.length;
            while(ds.isAutonomous() &&  ds.isEnabled()){
                for (int i = start; i < finished; i++){
                    if (ds.isEnabled() && runAutonomous){
                    switch (currentAutonomous[i][0]){
                        case DRIVES_GO_FORWARD:
                            log.logMessage("Auto Drives Foward");
                            drives.driveStraight(currentAutonomous[i][1]);
                            break;
                        case DRIVES_GO_REVERSE:
                            log.logMessage("Auto Drives Reverse");
                            drives.driveStraight(currentAutonomous[i][1]);
                            break;
                        case DRIVES_TURN_LEFT:
                            log.logMessage("Auto Turn Left");
                            drives.turn(-currentAutonomous[i][1]);
                            break;
                        case DRIVES_TURN_RIGHT:
                            log.logMessage("Auto Turn Right");
                            drives.turn(currentAutonomous[i][1]);
                            break;
                        case DRIVES_STOP:
                            
                            break;
                        case DRIVES_DONE:
                            log.logMessage("Auto Waiting for Drives");
                            isDoneDrives();
                            break;
                        case ACQ_READY:
                            log.logMessage("Auto is configuring");
                            isAcquisitionsReady();
                            break;
                        case ACQ_AQUIRE_BALL:
                            log.logMessage("Auto Acquiring");
                            acq.setMode(Acquisitions.AcqState.ACQUIRING);
                            break;
                        case ACQ_REVERSE:
                            log.logMessage("Auto Ejecting Ball");
                            acq.setMode(Acquisitions.AcqState.EJECT_BALL);
                            break;
                        case ACQ_ACQUIRE_IN_POSITION://ONLY WORKS WITH ACQUIRING
                            log.logMessage("Auto is waiting for Acquisitions to acquire");
                            isAcquisitionsDone(Acquisitions.AcqState.ACQUIRING);
                            break;
                        case ACQ_DONE:
                            log.logMessage("Auto is waiting for Acquisitions");
                            isAcquisitionsDone(currentAutonomous[i][1]);
                            break;
                        case SHOOTER_SHOOT:
                            log.logMessage("Shoting Ball");
                            shooter.shoot();
                            break;
                        case SHOOTER_SET_PRESET:
                            log.logMessage("Setting Preset");
                            acq.setMode(Acquisitions.AcqState.READY_TO_SHOOT);
                            break;
                        case SHOOTER_READY_TO_SHOOT:
                            log.logMessage("Shooter in Position");
                            isAcquisitionsDone(Acquisitions.AcqState.READY_TO_SHOOT);
                            break;
                        case SHOOTER_READY:
                            isShooterReady();
                            break;
                        case VISION_DISTANCE:
                            visionDistance = vision.getDistance();
                            log.logMessage("Vision getting Distance");
                            break;
                        case VISION_ANGLE:
                            visionAngle = vision.getDegrees();
                            log.logMessage("Vision getting Degrees");
                            break;
                        case VISION_HOT_TARGET:
                            visionHotGoal = vision.isHotGoal();
                            break;
                        case NEXT:
                            if(loopTime > 1){
                                i = (i - currentAutonomous[i][1]) - 1;//the extra one is to cancel the +1 for the loop
                                loopTime--;
                            }
                            log.logMessage("Next Loop");
                            break;
                        case LOOP:
                            loopTime = currentAutonomous[i][1];
                            log.logMessage("Setting Loop to " + currentAutonomous[i][1] + " loops");
                            break;
                        case WAIT:
                            try {
                                Thread.sleep(currentAutonomous[i][1]);
                            } catch (InterruptedException ex) {
                                ex.printStackTrace();
                            }
                            break;
                        case END:
                            runAutonomous = false;
                            log.logMessage("Auto has stopped ****************");
                            break;
                        default:
                            log.logMessage("No case statement: " + currentAutonomous[i]);
                            break;
                    }
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException ex) {
                        log.logError("AUTO: SLEEP FAILED");
                    }
                    }
                }       
            }
    }

    /**
     * Gets an instance of all the subsystems
     */
    public void init() {
        drives = Drives.getInstance();
        vision = Vision.getInstance();
        autoSelectSwitch = new AnalogChannel(IO.DEFAULT_SLOT, IO.AUTOSWITCH_CHANNEL);
        acq = Acquisitions.getInstance();
        shooter = Shooter.getInstance();
    }

    /**
     * Main run method. Called by GenericSubsystem
     * @throws Exception 
     */
    public void execute() throws Exception {
        if(ds.isAutonomous() && ds.isEnabled()){
            auto.runAutonomous();
        }else{
            auto.getAutoMode();
        }
    }
    
    /**
     * Waits until the Drives class is done doing its last command
     */
    private void isDoneDrives(){
        while(!drives.isLastCommandDone() && ds.isAutonomous()){
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
    
    /**
     * Waits until the Vision class is done doing its last command
     */
    private void isVisionDone(){
        while(!vision.isLastCommandDone() && ds.isAutonomous()){
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
    
    /**
     * Waits until acquisitions is ready for the next command
     */ 
    private void isAcquisitionsReady(){
        while(!acq.isAcquisitionsReady()){
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
    
    /**
     * Waits until acquisitions is in wantedDoneState state
     * @param wantedDoneState - the state to wait until acquisitions is in
     */
    private void isAcquisitionsDone(int wantedDoneState){
        while(!acq.isLastCommandDone(wantedDoneState)){
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
    
    private void isShooterReady(){
        while(!shooter.isLastCommandDone()){
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
    
    /**
     * Sets if autonomous is allowed to run
     * @param allowedToRun - whether or not autonomous is ready to run
     */ 
    public void runAuto(boolean allowedToRun){
        runAutonomous = allowedToRun;
    }
    
    /**
     * Sends the name of the current autonomous to the livewindow and gets if we
     * are using smart dashboard to choose the autonomous mode
     * @param autoName - the name of the current autonomous mode
     */ 
    private void sendSmartAuto(String autoName){
        SmartDashboard.putString("Selected Auto Mode: ", autoName);
        smartAutoMode = SmartDashboard.getBoolean(smartChooser);
    }

    /**
     * Inits and sends all of the components of the livewindow
     */ 
    public void liveWindow() {
        smartChoose = new SendableChooser();
        smartChoose.addDefault("No Auto", new Integer(0));
        smartChoose.addObject("Auto 1", new Integer(1));
        smartChoose.addObject("Auto 2", new Integer(2));
        smartChoose.addObject("Auto 3", new Integer(3));
        smartChoose.addObject("Auto 4", new Integer(4));
        smartChoose.addObject("Auto 5", new Integer(5));
        smartChoose.addObject("Auto 6", new Integer(6));
        smartChoose.addObject("Auto 7", new Integer(7));
        smartChoose.addObject("Auto 8", new Integer(8));
        SmartDashboard.putData("Auto Mode", smartChoose);
        SmartDashboard.putBoolean(smartChooser, false);
    }

    public int sleepTime() {
        return 20;
    }

    /**
     * No regular info to log about autonomous.
     */ 
    public void logInfo() {
        
    }
}
