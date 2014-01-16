/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.gosparx.sensors;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 *
 * @author Connor
 */
public class LightSensor {
    private DigitalOutput led;
    private AnalogChannel lightReader;
    public final int BLUE_VALUE = 0;//TODO:find value
    public final int RED_VALUE = 0;//TODO:find value
    public final int WHITE_VALUE = 0;//TODO:find value
    int colorValue = 0;
    
    
    public LightSensor(int lightSlot, int lightChannel, int readerSlot, int readerChannel){
        led = new DigitalOutput(lightSlot, lightChannel);
        led.set(true);//turns light on
        lightReader = new AnalogChannel(readerSlot, readerChannel);
    }
    
    /**
     * 
     * @return the actual value of the light sensor 
     */
    public int getValue(){
        return lightReader.getValue();
    }
    
    /**
     * Sees if the current light value is the same as a predetermined constant
     * @return the color the sensor sees 
     */
    public int getColor(){
        colorValue = lightReader.getValue();
        if(colorValue > WHITE_VALUE){
            return WHITE_VALUE;
        }else if(colorValue > RED_VALUE){
            return RED_VALUE;
        }else if(colorValue > BLUE_VALUE){
            return BLUE_VALUE;
        }else{
            return 0;
        }
    }
    
    /**
     * @param color - a constant
     * @return true if the color given is the same color seen by the sensor
     */
    public boolean isColor(int color){
        return(getColor() == color);
    }
}
