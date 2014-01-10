package org.gosparx.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author Alex
 * @date 1/08/14
 * @version 1.0
 */
public class Logger {
    private LogWriter writer;
    private String subsystem;
    private DriverStation ds = DriverStation.getInstance();
    
    /**
     * Creates a new Logger to log
     * @param subsystem The desired prefix for the log messages
     */
    public Logger(String subsystem){
        this.subsystem = subsystem;
        writer = LogWriter.getInstance();
    }
    
    /**
     * Logs a message in the following format: 
     * "[Time into the match] {Current mode} Subsystem: message"
     * followed by a new line
     * @param message the desired message to log
     */
    public void logMessage(String message){
        String mode;
        if(ds.isAutonomous()){
            mode = "Auto";
        }else if(ds.isOperatorControl()){
            mode = "Teleop";
        }else{
            mode = "Disabled";
        }
        String toWrite = "[" + ds.getMatchTime() + "] {" + mode + "} " + subsystem + ": " + message + "/n";
        writer.log(toWrite);
    }
}
