package org.gosparx.util;

import com.sun.squawk.microedition.io.FileConnection;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.networktables2.util.List;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import javax.microedition.io.Connector;
import org.gosparx.subsystem.GenericSubsystem;

/**
 * @author Alex
 * @date 1/08/14
 */

public class LogWriter extends GenericSubsystem{
    private static LogWriter writer;
    private final List messagesToLog;
    private FileConnection fileCon;
    private FileConnection fileConConfig;
    private DataOutputStream dos;
    private DataOutputStream dosConfig;
    private DataInputStream dis;
    private final String configPath = "file:///loggingConfig.txt";
    private final int MAX_LOGS = 5;
    private String[] prevMessages  = new String[6];
    private DriverStationLCD dsLCD = DriverStationLCD.getInstance();
    
    public static int LEVEL_DEBUG                                           = 0;
    public static int LEVEL_ERROR                                           = 1;
   
    /**
     * Returns the singleton LogWriter
     * @return the singleton LogWriter
     */
    public static LogWriter getInstance(){
        if(writer == null){
            writer = new LogWriter();
        }
        return writer;
    }
    /**
     * Makes a new LogWriter
     */
    private LogWriter(){
        super("LogWriter", Thread.NORM_PRIORITY);
        messagesToLog = new List();
    }
    /**
     * Logs the message to the file
     * @param message The message to log
     */
    public void log(String info, String message, int level){
        synchronized(messagesToLog){
            messagesToLog.add(new LogMessage(level, info, message));
            messagesToLog.notify();
        }
    }
    /**
     * Setups up the config file if it does not exist and sets the first log to
     * use as log0.txt. Otherwise it reads the last used logs and increments it
     * by 1. It wraps around.
     */
    public void init() {
        String emptyString = "";
        prevMessages[0] = emptyString;
        prevMessages[1] = emptyString;
        prevMessages[2] = emptyString;
        prevMessages[3] = emptyString;
        prevMessages[4] = emptyString;
        prevMessages[5] = emptyString;
        try {
            int toUse = 0;
            try {
                fileConConfig = (FileConnection)Connector.open(configPath, Connector.READ_WRITE);
                dis = fileConConfig.openDataInputStream();
                dosConfig = fileConConfig.openDataOutputStream();
                char lastUsedChar = (char) dis.read();
                String toParse = "" + lastUsedChar;
                toUse = Integer.parseInt(toParse) + 1;
                toUse %= MAX_LOGS;
                String toWrite = "" + toUse;
                try{
                    fileConConfig.delete();
                    fileConConfig.create();
                    dosConfig = fileConConfig.openDataOutputStream();
                    dosConfig.write(toWrite.getBytes());
                    dosConfig.close();
                    dis.close();
                    fileConConfig.close();
                }catch(IOException e){
                }
            }catch(IOException e){
                if(fileConConfig.fileSize() != 0){
                fileConConfig.create();
                dosConfig = fileConConfig.openDataOutputStream();
                String toWrite = "" + toUse;
                dosConfig.write(toWrite.getBytes());
                dosConfig.close();
                }
            }
            fileCon = (FileConnection)Connector.open("file:///log" + toUse + ".txt", Connector.READ_WRITE);
            if(fileCon.exists()){
                fileCon.delete();
            }
            fileCon.create();
            fileCon = (FileConnection)Connector.open("file:///log" + toUse + ".txt", Connector.READ_WRITE);
            dos = fileCon.openDataOutputStream();
        } catch (IOException ex) {
        }
    }
    /**
     * Logs the first message to log in the queue every 20 ms
     */
    public void execute() throws Exception {
        String message, info;
        LogMessage logMessage;
        int level;
        if(true){
            synchronized(messagesToLog){
                if(messagesToLog.isEmpty())
                    messagesToLog.wait();
                
                logMessage = (LogMessage) messagesToLog.get(0);
                message = logMessage.getMessage();
                level = logMessage.getLevel();
                info = logMessage.getInfo();
                messagesToLog.remove(0);
            }
            String toWrite = info + message + "\n";
            dos.write(toWrite.getBytes());
            System.out.print(toWrite);
            if(level == LEVEL_ERROR){
                prevMessages[0] = prevMessages[1];
                prevMessages[1] = prevMessages[2];
                prevMessages[2] = prevMessages[3];
                prevMessages[3] = prevMessages[4];
                prevMessages[4] = info;
                prevMessages[5] = message;
                dsLCD.println(DriverStationLCD.Line.kUser1, 1, prevMessages[0]);
                dsLCD.println(DriverStationLCD.Line.kUser2, 1, prevMessages[1]);
                dsLCD.println(DriverStationLCD.Line.kUser3, 1, prevMessages[2]);
                dsLCD.println(DriverStationLCD.Line.kUser4, 1, prevMessages[3]);
                dsLCD.println(DriverStationLCD.Line.kUser5, 1, prevMessages[4]);
                dsLCD.println(DriverStationLCD.Line.kUser6, 1, prevMessages[5]);
                dsLCD.updateLCD();
            }
        }
    }

    public void liveWindow() {
       
    }

    public int sleepTime() {
        return 50;
    }

    public void logInfo() {
        
    }
}
