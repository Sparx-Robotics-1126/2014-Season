package org.gosparx.util;

import com.sun.squawk.microedition.io.FileConnection;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import javax.microedition.io.Connector;
import org.gosparx.subsystem.GenericSubsystem;

/**
 * @author Alex
 * @date 1/08/14
 * @version 1
 */
public class LogWriter{
    private static LogWriter writer;
    private FileConnection logFile;
    private DataOutputStream dos;
    public static LogWriter getInstance(){
        if(writer == null){
            writer = new LogWriter();
        }
        return writer;
    }
    
    public LogWriter(){
        try {
            logFile = (FileConnection)Connector.open("file:///log.txt", Connector.READ_WRITE);
            dos = logFile.openDataOutputStream();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    
    public void log(String message){
        try {
            dos.write(message.getBytes(), 0, message.getBytes().length);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
