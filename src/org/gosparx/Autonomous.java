package org.gosparx;

import edu.wpi.first.wpilibj.DriverStation;
import org.gosparx.subsystem.Drives;
import org.gosparx.subsystem.GenericSubsystem;
import org.gosparx.subsystem.Vision;


public class Autonomous extends GenericSubsystem{
    private Autonomous auto;
    private Drives drives;
    private Vision vision;
    private int[][] currentAutonomous;
    
    private boolean runAutonomous = false;
    
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

    /* Misc */
    private static final int LOOP                           = 97;//number of loops, 
    private static final int WAIT                           = 98;
    private static final int END                            = 99;
    /**************************************************************************/
    /**************************** End of commands *****************************/
    /**************************************************************************/
    
    public static final int[][] noAuto = { 
        
    };
    
    private Autonomous(){
        super("Autonomous", GenericSubsystem.NORM_PRIORITY);
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
    
    public void run(){
        int start = 0, current = start, finished = currentAutonomous.length;
        while (true){
            while(ds.isAutonomous() &&  ds.isEnabled()){
                    current++;
                for (int i = start; i <= finished; i++){
                    if (ds.isEnabled() && runAutonomous){
                    switch (currentAutonomous[i][0]){
                        case DRIVES_GO_FORWARD:
                            throw new java.lang.RuntimeException("Drive Foward hasen't been created!!!");
                            //break;
                        case DRIVES_GO_REVERSE:
                            throw new java.lang.RuntimeException("Drive Reverse hasen't been created!!!");
                            //break;
                        case DRIVES_TURN_LEFT:
                            throw new java.lang.RuntimeException("Drive Left hasen't been created!!!");
                            //break;
                        case DRIVES_TURN_RIGHT:
                            print("Turning Right");
                        case DRIVES_STOP:
                            print("Stopping");
                            //break;
                        case DRIVES_DONE:
                            print("DriveDone() was called");
                            //break;
                        case INTAKE_AQUIRE_BALL:
                            //break;
                        case INTAKE_REVERSE:
                            //break;
                        case INTAKE_IN_POSITION:
                            //break;
                        case INTAKE_DONE:
                            //break;
                        case SHOOTER_SHOOT:
                            //break;
                        case SHOOTER_SET_PRESET:
                            //break;
                        case SHOOTER_IN_POSITION:
                            //break;
                        case SHOOTER_DONE:
                            //break;
                        case LOOP:
                            //break;
                        case WAIT:
//                            wait(currentAutonomous[i][1]);
                            //break;
                        case END:
                            runAutonomous = false;
                        default:
//                            print("No case statement: " + currentAutonomous[i]);
                    }
                } 
                sleep(30);   
            }
        }            
        sleep(30);   
      }
    }

    public void init() {
        throw new java.lang.UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    public void execute() throws Exception {
        throw new java.lang.UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
}
