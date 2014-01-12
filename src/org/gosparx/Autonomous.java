package org.gosparx;

import edu.wpi.first.wpilibj.DriverStation;
import org.gosparx.subsystem.Drives;
import org.gosparx.subsystem.Vision;


public class Autonomous {
    private Autonomous auto;
    private Drives drives;
    private Vision vision;
    private int[][] currentAutonomous;
    
    private boolean runAutonomous = false;
    
    /**************************************************************************/
    /*************************Manual Switch Voltages **************************/
    /**************************************************************************/
    public static final double AUTO_SETTING_1 = 3.208;
    public static final double AUTO_SETTING_2 = 3.126;
    public static final double AUTO_SETTING_3 = 3.036;
    public static final double AUTO_SETTING_4 = 2.935;
    public static final double AUTO_SETTING_5 = 2.824;
    public static final double AUTO_SETTING_6 = 2.701;
    public static final double AUTO_SETTING_7 = 2.563;
    public static final double AUTO_SETTING_8 = 2.405;
    public static final double AUTO_SETTING_9 = 2.225;
    /**************************************************************************/
    /************************ Autonomous commands *****************************/
    /**************************************************************************/
    
    /* Drives */
    public static final int DRIVES_GO_FORWARD               = 1;//distance(inches), speed (int)
    public static final int DRIVES_GO_REVERSE               = 2;
    public static final int DRIVES_TURN_RIGHT               = 3;
    public static final int DRIVES_TURN_LEFT                = 4;
    public static final int DRIVES_STOP                     = 5;
    public static final int DRIVES_DONE                     = 9;

    /* Misc */
    public static final int WAIT                            = 90;
    public static final int END                             = 99;
    /**************************************************************************/
    /**************************** End of commands *****************************/
    /**************************************************************************/
    
    public static final int[][] noAuto = { 
        
    };
    
    private Autonomous(){
//        drives = Drives.getInstance();
        drives = new Drives();//getInstance hasen't been implemented
        vision = Vision.getInstance();
        
    }
    
    public Autonomous getInstance(){
        if(auto == null){
            auto = new Autonomous();
        }
        return auto;
    }
    
    public void getAutoMode(){
           double voltage = 0; // need voltage reaading;
           if (voltage >= AUTO_SETTING_1){
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
               currentAutonomous = null;
           }else if (voltage >= AUTO_SETTING_7){
               currentAutonomous = noAuto;
           }else{
               currentAutonomous = noAuto;
           }
//        }
    }
    
    public void run(){
        getAutoMode();
        int start = 0, current = start, finished = currentAutonomous.length;
//        print("Length of Array: " + finished);
        while (true){
            while(getLocalMode() == SubSystem.AUTO &&  isEnabled()){
                    current++;
                for (int i = start; i <= finished; i++){
                    if (isEnabled() && runAutonomous){
                    switch (currentAutonomous[i][0]){
                        case DRIVES_GO_FORWARD:
                            print("Driving forward");
                            break;
                        case DRIVES_GO_REVERSE:
                            print("Driving reverse");
                            break;
                        case DRIVES_TURN_LEFT:
                            print("Turning Left");
                            break;
                        case DRIVES_TURN_RIGHT:
                            print("Turning Right");
                        case DRIVES_STOP:
                            print("Stopping");
                            break;
                        case DRIVES_DONE:
                            print("DriveDone() was called");
                            break;
                            
                        case WAIT:
                            wait(currentAutonomous[i][1]);
                            break;
                        case END:
                            runAutonomous = false;
                        default:
                            print("No case statement: " + currentAutonomous[i]);
                    }
                } 
                sleep(30);   
            }
        }            
        sleep(30);   
      }
    }
}
