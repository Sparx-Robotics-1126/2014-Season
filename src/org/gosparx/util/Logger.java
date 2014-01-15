package org.gosparx.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;

/**
 * @author Alex
 * @date 1/08/14
 */
public class Logger {
    private LogWriter writer;
    private String subsystem;
    private DriverStation ds = DriverStation.getInstance();
    private final int DIGITS_IN_TIME = 8;
    private final int PRECISION = 4;
    
    public static String SUB_DRIVES = "Drive";
    public static String SUB_SHOOTER = "Shoot";
    public static String SUB_ACQUISITIONS = "Acqui";
    public static String SUB_VISON = "Vison";
    
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
            mode = "Aut";
        }else if(ds.isOperatorControl()){
            mode = "Tel";
        }else{
            mode = "Dis";
        }
        double time = ds.getMatchTime();
        time *= com.sun.squawk.util.MathUtils.pow(10, PRECISION);
        int timeInt = (int)time;
        String timeToFormat = "" + timeInt;
        String timeFormatted = timeToFormat;
        if(timeToFormat.length()<= DIGITS_IN_TIME) {
            timeFormatted = "0000000000000000".substring(0, DIGITS_IN_TIME - timeToFormat.length()) + timeInt;
        }
        timeFormatted = timeFormatted.substring(0,timeFormatted.length() - PRECISION) + "." + timeFormatted.substring(timeFormatted.length() - PRECISION);
        String info = "(DEBUG)" + "[" + timeFormatted + "] {" + mode + "} " + subsystem + ": ";
        writer.log(info, message, LogWriter.LEVEL_DEBUG);
    }
    
    public void logError(String message){
        String mode;
        if(ds.isAutonomous()){
            mode = "Aut";
        }else if(ds.isOperatorControl()){
            mode = "Tel";
        }else{
            mode = "Dis";
        }
        double time = ds.getMatchTime();
        time *= com.sun.squawk.util.MathUtils.pow(10, PRECISION);
        int timeInt = (int)time;
        String timeToFormat = "" + timeInt;
        String timeFormatted = timeToFormat;
        if(timeToFormat.length()<= DIGITS_IN_TIME) {
            timeFormatted = "0000000000000000".substring(0, DIGITS_IN_TIME - timeToFormat.length()) + timeInt;
        }
        timeFormatted = timeFormatted.substring(0,timeFormatted.length() - PRECISION) + "." + timeFormatted.substring(timeFormatted.length() - PRECISION);
        String info = "(ERROR)" + "[" + timeFormatted + "]{" + mode + "}" + subsystem + ": ";
        writer.log(info, message, LogWriter.LEVEL_ERROR);;
    }
}