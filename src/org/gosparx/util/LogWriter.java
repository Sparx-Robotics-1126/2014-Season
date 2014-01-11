package org.gosparx.util;

import com.sun.squawk.microedition.io.FileConnection;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import javax.microedition.io.Connector;

/**
 * @author Alex
 * @date 1/08/14
 * @version 1
 * Fix the stupid error
 */

public class LogWriter{
    private static LogWriter writer;
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
        try {
            int toUse = 0;
            try {
                fileConConfig = (FileConnection)Connector.open(configPath, Connector.READ_WRITE);
                dis = fileConConfig.openDataInputStream();
                dosConfig = fileConConfig.openDataOutputStream();
                char lastUsedChar = dis.readChar();
                String toParse = "" + lastUsedChar;
                toUse = Integer.parseInt(toParse) + 1;
                if(toUse >= 5){
                    toUse = 0;
                }
                String toWrite = "" + toUse;
                try{
                    fileConConfig.delete();
                    fileConConfig.create();
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
            dos = fileCon.openDataOutputStream();
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
        }else{
            try {
                throw new Exception("The DataOutputStream is null!");
            } catch (Exception ex) {
                ex.printStackTrace();
            }
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
}
