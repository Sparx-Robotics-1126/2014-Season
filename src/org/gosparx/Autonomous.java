package org.gosparx;

import edu.wpi.first.wpilibj.DriverStation;
import org.gosparx.subsystem.Drives;
import org.gosparx.subsystem.GenericSubsystem;
import org.gosparx.subsystem.Vision;


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
    private double visionDistance = 0;
    
    /**
     * The angle from the camera to the target (closest one)
     */
    private int visionAngle = 0;
    
    /**
     * Test to see if the closest target is the hot goal
     */
    private boolean visionHotGoal = false;

    
    /**************************************************************************/
    /*************************Manual Switch Voltages **************************/
    /**************************************************************************/
    private static final double AUTO_SETTING_0 = 3.208;
    private static final double AUTO_SETTING_1 = 3.126;
    private static final double AUTO_SETTING_2 = 3.036;
    private static final double AUTO_SETTING_3 = 2.935;
    private static final double AUTO_SETTING_4 = 2.824;
    private static final double AUTO_SETTING_5 = 2.701;
    private static final double AUTO_SETTING_6 = 2.563;
    private static final double AUTO_SETTING_7 = 2.405;
    private static final double AUTO_SETTING_8 = 2.225;
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
    private static final int INTAKE_AQUIRE_BALL             = 10;
    private static final int INTAKE_REVERSE                 = 11;
    private static final int INTAKE_IN_POSITION             = 12;
    private static final int INTAKE_DONE                    = 13;
    
    /* Shooter */
    private static final int SHOOTER_SHOOT                  = 20;
    private static final int SHOOTER_SET_PRESET             = 21;
    private static final int SHOOTER_IN_POSITION            = 22;
    private static final int SHOOTER_DONE                   = 23;
    
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
    private static final int[][] noAuto = { 
        {END}
    };
    
    /**
     * Drives forward 20 feet
     */
    private static final int[][] moveFoward = {
        {DRIVES_GO_FORWARD, 20*12},  
        {DRIVES_DONE},
        {END}
    };
    
    /**
     * Drives in a 4x4 foot square, turning to the right
     */
    private static final int[][] autoSquare = {
        {LOOP, 4*2},
        {DRIVES_GO_FORWARD, 12*4},
        {DRIVES_DONE},
        {DRIVES_TURN_RIGHT, 90},
        {DRIVES_DONE},
        {NEXT, 4},
        {END}
    };
    
    /**
     * Camera will follow the target
     */
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
    private static final int[][] turn90 = {
        {DRIVES_TURN_LEFT, 90},
        {DRIVES_DONE},
        {END}
    };
   
    /**
     * Autonomous Constructor
     */
    private Autonomous(){
        super("Autonomous", GenericSubsystem.NORM_PRIORITY);       
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
           double voltage = 0; // need voltage reaading;
           if (voltage >= AUTO_SETTING_0){
               currentAutonomous = null;
           }else if (voltage >= AUTO_SETTING_1){
               currentAutonomous = null;
           }else if (voltage >= AUTO_SETTING_2){
               currentAutonomous = null;
           }else if (voltage >= AUTO_SETTING_3){
               currentAutonomous = null;
           }else if (voltage >= AUTO_SETTING_4){
               currentAutonomous = null;
           }else if (voltage >= AUTO_SETTING_5){
               currentAutonomous = null;
           }else if (voltage >= AUTO_SETTING_6){
               currentAutonomous = noAuto;
           }else{
               currentAutonomous = noAuto;
           }
    }
    
    /**
     * Gets the data from the array and tells each subsystem what actions to take.
     */
    private void runAutonomous(){
        currentAutonomous = turn90;
        int start = 0, current = start, finished = currentAutonomous.length;
        while (true){
            while(ds.isAutonomous() &&  ds.isEnabled()){
                    current++;
                for (int i = start; i <= finished; i++){
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
                        case INTAKE_AQUIRE_BALL:
                            
                            break;
                        case INTAKE_REVERSE:
                            
                            break;
                        case INTAKE_IN_POSITION:
                            
                            break;
                        case INTAKE_DONE:
                            
                            break;
                        case SHOOTER_SHOOT:
                            
                            break;
                        case SHOOTER_SET_PRESET:
                            
                            break;
                        case SHOOTER_IN_POSITION:
                            
                            break;
                        case SHOOTER_DONE:
                            isVisionDone();
                            break;
                        case VISION_DISTANCE:
                            visionDistance = vision.getDistance();
                            break;
                        case VISION_ANGLE:
                            visionAngle = vision.getLocation();
                            break;
                        case VISION_HOT_TARGET:
                            visionHotGoal = vision.isHotGoal();
                            break;
                        case DRIVES_TRACK_TARGET:
                            System.out.println("Distance: " + visionDistance + "  Location: " + visionAngle);
                            if(visionAngle > 190){
                                drives.setSpeed(5, -5);
                            }else if(visionAngle < 170){
                                drives.setSpeed(-5, 5);
                            }else if(visionDistance > 20){
                                drives.setSpeed(5, 5);
                            }else if(visionDistance < 15){
                                drives.setSpeed(-5, -5);
                            }else{
                                drives.setSpeed(0, 0);
                            }
                            break;
                        case NEXT:
                            if(loopTime > 0){
                                i = i - currentAutonomous[i][1] - 1;//the extra one is to cancel the +1 for the loop
                                loopTime--;
                            }
                            break;
                        case LOOP:
                            loopTime = currentAutonomous[i][1];
                            break;
                        case WAIT:
                            
                            break;
                        case END:
                            runAutonomous = false;
                        default:
//                            print("No case statement: " + currentAutonomous[i]);
                    }
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
    }

    /**
     * Main run method. Called by GenericSubsystem
     * @throws Exception 
     */
    public void execute() throws Exception {
        while(true){
            Thread.sleep(20);
            if(ds.isAutonomous() && ds.isEnabled()){
                auto.runAutonomous();
            }else{
                auto.getAutoMode();
            }
        }
    }
    
    /**
     * Waits until the Drives class is done doing its last command
     */
    private void isDoneDrives(){
        while(!drives.isLastCommandDone()){
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
        while(!vision.isLastCommandDone()){
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
    
    public void runAuto(boolean allowedToRun){
        runAutonomous = allowedToRun;
    }
}
