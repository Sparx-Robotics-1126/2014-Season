package org.gosparx;

import edu.wpi.first.wpilibj.DriverStation;
import org.gosparx.subsystem.Drives;
import org.gosparx.subsystem.GenericSubsystem;
import org.gosparx.subsystem.Vision;


public class Autonomous extends GenericSubsystem{
    private static Autonomous auto;
    private Drives drives;
    private Vision vision;
    private int[][] currentAutonomous;
    
    private boolean runAutonomous = true;
    private int loopTime = 0;
    private double visionDistance = 0;
    private int visionAngle = 0;
    private boolean visionHotGoal = false;
    private boolean firstLoop = true;
    
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
    private static final int DRIVES_TURN_RIGHT              = 3;
    private static final int DRIVES_TURN_LEFT               = 4;
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
    private static final int VISION_HOT_TARGET          = 32;
    
    private static final int CUSTOM_1                       = 40;

    /* Misc */
    private static final int LOOP                           = 97;//number of loops, 
    private static final int WAIT                           = 98;
    private static final int END                            = 99;
    /**************************************************************************/
    /**************************** End of commands *****************************/
    /**************************************************************************/
    
    public static final int[][] noAuto = { 
        
    };
    
    public static final int[][] cameraFollow = { 
        {LOOP, Integer.MAX_VALUE},
        {CUSTOM_1},
        {END}
    };
    
    private Autonomous(){
        super("Autonomous", GenericSubsystem.NORM_PRIORITY);       
    }
    
    public static Autonomous getInstance(){
        if(auto == null){
            auto = new Autonomous();
        }
        return auto;
    }
    
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
    
    private void runAutonomous(){
        currentAutonomous = cameraFollow;
        int start = 0, current = start, finished = currentAutonomous.length;
        while (true){
            while(ds.isAutonomous() &&  ds.isEnabled()){
                    current++;
                for (int i = start; i <= finished; i++){
                    if (ds.isEnabled() && runAutonomous){
                    switch (currentAutonomous[i][0]){
                        case DRIVES_GO_FORWARD:
                            
                            break;
                        case DRIVES_GO_REVERSE:
                            
                            break;
                        case DRIVES_TURN_LEFT:
                            
                            break;
                        case DRIVES_TURN_RIGHT:
                            
                            break;
                        case DRIVES_STOP:
                            
                            break;
                        case DRIVES_DONE:
                            
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
                        case CUSTOM_1:
                            visionDistance = vision.getDistance();
                            visionAngle = vision.getLocation();
                            System.out.println("Distance: " + visionDistance + "  Location: " + visionAngle);
                            if(visionAngle > 190){
                                drives.setSpeed(36, -36);
                            }else if(visionAngle < 170){
                                drives.setSpeed(-36, 36);
                            }else if(visionDistance > 14){
                                drives.setSpeed(-36, -36);
                            }else if(visionDistance < 10){
                                drives.setSpeed(36, 36);
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
                    if(loopTime > 0 && !firstLoop){
                        i = i - 1;
                        loopTime = loopTime - 1;
                    }else if(firstLoop){
                        firstLoop = false;
                    }else{
                        firstLoop = true;
                    }
            }
        }              
      }
    }

    public void init() {
        drives = Drives.getInstance();
        vision = Vision.getInstance();
    }

    public void execute() throws Exception {
        System.out.println("I HAVE STARTED");
        while(true){
            Thread.sleep(20);
            if(ds.isAutonomous() && ds.isEnabled()){
                auto.runAutonomous();
            }
        }
    }
}
