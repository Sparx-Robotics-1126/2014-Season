package org.gosparx.util;

import com.sun.squawk.microedition.io.FileConnection;
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
        super("LogWriter", Thread.MIN_PRIORITY);
        messagesToLog = new List();
    }
    /**
     * Logs the message to the file
     * @param message The message to log
     */
    public void log(String message){
        synchronized(messagesToLog){
            messagesToLog.add(message);
            messagesToLog.notify();
        }
    }
    /**
     * Closes and sets to null both the FileConnector and DataOutputStream
     */
    public void close(){
        if(dos != null && fileCon != null){
            try {
                dos.close();
                fileCon.close();
            } catch (IOException ex) {
                ex.printStackTrace();
            }
            dos = null;
            fileCon = null;
        }
    }
    /**
     * Temporally pauses logging by closing the streams
     */
    public void pause(){
        try {
            dos.close();
            fileCon.close();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    /**
     * Resumes paused file logging
     */
    public void resume(){
        try {
            fileCon.create();
            dos = fileCon.openDataOutputStream();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public void init() {
        try {
            int toUse = 0;
            try {
                fileConConfig = (FileConnection)Connector.open(configPath, Connector.READ_WRITE);
                dis = fileConConfig.openDataInputStream();
                dosConfig = fileConConfig.openDataOutputStream();
                char lastUsedChar = (char) dis.read();
                String toParse = "" + lastUsedChar;
                toUse = Integer.parseInt(toParse) + 1;
                toUse %= 5;
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
                    e.printStackTrace();
                }
            }catch(IOException e){
                e.printStackTrace();
                fileConConfig.create();
                dosConfig = fileConConfig.openDataOutputStream();
                String toWrite = "" + toUse;
                dosConfig.write(toWrite.getBytes());
                dosConfig.close();
            }
            fileCon = (FileConnection)Connector.open("file:///log" + toUse + ".txt", Connector.READ_WRITE);
            if(fileCon.exists()){
                fileCon.delete();
            }
            fileCon.create();
            fileCon = (FileConnection)Connector.open("file:///log" + toUse + ".txt", Connector.READ_WRITE);
            dos = fileCon.openDataOutputStream();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    /**
     * Logs the first message to log in the queue every 20 ms
     */
    public void execute() throws Exception {
        String message;
        while (true) {
            synchronized(messagesToLog){
                if(messagesToLog.isEmpty())
                    messagesToLog.wait();
                
                message = (String) messagesToLog.get(0);
                messagesToLog.remove(0);
            }
            dos.write(message.getBytes(), 0, message.getBytes().length);
            Thread.sleep(20);
        }
    }
}
