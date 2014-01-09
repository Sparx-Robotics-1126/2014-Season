package org.gosparx.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.gosparx.subsystem.GenericSubsystem;

/**
 * @author Alex
 * @date 1/08/14
 * @version 1.0
 */
public class Logger {
    private LogWriter writer;
    private String subsystem;
    private DriverStation ds = DriverStation.getInstance();
    //[Comp Time] {Current Mode} Subsystem: Message
    public Logger(String subsystem){
        this.subsystem = subsystem;
        writer = LogWriter.getInstance();
    }
    
    public void logMessage(String message){
        String mode;
        if(ds.isAutonomous()){
            mode = "Auto";
        }else if(ds.isOperatorControl()){
            mode = "Teleop";
        }else{
            mode = "Disabled";
        }
        String toWrite = "[" + ds.getMatchTime() + "] {" + mode + "} " + subsystem + ": " + message;
        writer.log(toWrite);
    }
}
