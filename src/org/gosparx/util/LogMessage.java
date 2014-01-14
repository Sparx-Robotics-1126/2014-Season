/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.gosparx.util;

/**
 *
 * @author Alex_Fixed
 */
public class LogMessage {
    private int level;
    private String message;
    private String info;
    public LogMessage(int level, String info, String message){
        this.level = level;
        this.message = message;
        this.info = info;
    }
    public int getLevel(){
        return level;
    }
    public String getMessage(){
        return message;
    }
    public String getInfo(){
        return info;
    }
}
