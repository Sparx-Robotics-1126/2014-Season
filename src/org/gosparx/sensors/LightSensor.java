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
    private AnalogChannel blueReader;
    private AnalogChannel redReader;
    private AnalogChannel greenReader;
    public final int BLUE_VALUE = 0;//TODO:find value
    public final int RED_VALUE = 0;//TODO:find value
    public final int WHITE_VALUE = 0;//TODO:find value
    int blueValue = 0;
    int redValue = 0;
    int greenValue = 0;
    
    
    public LightSensor(int lightSlot, int lightChannel, int blueSlot, int blueChannel, int greenSlot, int greenChannel, int redSlot, int redChannel){
        led = new DigitalOutput(lightSlot, lightChannel);
        led.set(true);//turns light on
        blueReader = new AnalogChannel(blueSlot, blueChannel);
        redReader = new AnalogChannel(redChannel, redChannel);
        greenReader = new AnalogChannel(greenSlot, greenChannel);
    }

    
    /**
     * Sees if the current light value is the same as a predetermined constant
     * @return the color the sensor sees 
     */
    public int getColor(){
        blueValue = blueReader.getValue();
        redValue = redReader.getValue();
        greenValue = greenReader.getValue();
        System.out.println("BLUE: " + blueValue + " RED: " + redValue + " GREEN: " + greenValue);
        return 1;
//        if(colorValue > WHITE_VALUE){
//            return WHITE_VALUE;
//        }else if(colorValue > RED_VALUE){
//            return RED_VALUE;
//        }else if(colorValue > BLUE_VALUE){
//            return BLUE_VALUE;
//        }else{
//            return 0;
//        }
    }
    
    /**
     * @param color - a constant
     * @return true if the color given is the same color seen by the sensor
     */
    public boolean isColor(int color){
        return(getColor() == color);
    }
}
