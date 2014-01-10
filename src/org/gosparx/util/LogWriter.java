package org.gosparx.util;

import com.sun.squawk.microedition.io.FileConnection;
import java.io.DataOutputStream;
import java.io.IOException;
import javax.microedition.io.Connector;

/**
 * @author Alex
 * @date 1/08/14
 * @version 1
 */
public class LogWriter{
    private static LogWriter writer;
    private FileConnection logFile;
    private DataOutputStream dos;
    
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
        try {
            logFile = (FileConnection)Connector.open("file:///log.txt", Connector.READ_WRITE);
            if(logFile.exists()){
                logFile.delete();
                logFile = (FileConnection)Connector.open("file:///log.txt", Connector.READ_WRITE);
            }
            logFile.create();
            if (logFile != null){
                dos = logFile.openDataOutputStream();
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    /**
     * Logs the message to the file
     * @param message The message to log
     */
    public void log(String message){
        if(dos != null){
            try {
              dos.write(message.getBytes(), 0, message.getBytes().length);
            } catch (IOException ex) {
               ex.printStackTrace();
            }
        }
    }
    /**
     * Closes and sets to null both the FileConnector and DataOutputStream
     */
    public void close(){
        if(dos != null && logFile != null){
            try {
                dos.close();
                logFile.close();
            } catch (IOException ex) {
                ex.printStackTrace();
            }
            dos = null;
            logFile = null;
        }
    }
    /**
     * Temporally pauses logging by closing the streams
     */
    public void pause(){
        try {
            dos.close();
            logFile.close();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    /**
     * Resumes paused file logging
     */
    public void resume(){
        try {
            logFile.create();
            dos = logFile.openDataOutputStream();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
