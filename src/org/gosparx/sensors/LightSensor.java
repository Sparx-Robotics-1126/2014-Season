package org.gosparx.sensors;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 *
 * @author Connor
 */
public class LightSensor {
    private AnalogChannel blueReader;
    private AnalogChannel redReader;
//    private AnalogChannel greenReader;//green is not used
    public static final int UNKNOWN_COLOR = 0;
    public static final int BLUE_COLOR = 1;//TODO:find value
    public static final int RED_COLOR = 2;//TODO:find value
    public static final int WHITE_COLOR = 3;//TODO:find value
    public static final int CARPET_COLOR = 4;
    private static final int WHITE_THRESHOLD = 60;//TODO: FIND VALUE
    private static final int CARPET_THRESHOLD = 20;
    private int blueValue = 0;
    private int redValue = 0;
    private int totalValue = 0;
    private static final int RED_THRESHOLD = 50;
    private static final int BLUE_THRESHOLD = 50;
    
    public LightSensor(int lightSlot, int lightChannel, int blueSlot, int blueChannel, int redSlot, int redChannel){
        new DigitalOutput(lightSlot, lightChannel).set(true);
        blueReader = new AnalogChannel(blueSlot, blueChannel);
        redReader = new AnalogChannel(redSlot, redChannel);
    }
    
    public LightSensor(int blueSlot, int blueChannel, int redSlot, int redChannel){
        blueReader = new AnalogChannel(blueSlot, blueChannel);
        redReader = new AnalogChannel(redSlot, redChannel);
    }
    
    /**
     * Returns a direct value from the color sensor
     */
    public int getRedValue(){
        return redReader.getValue();
    }
    
    public int getBlueValue(){
        return blueReader.getValue();
    }

    /**
     * Sees if the current light value is the same as a predetermined constant
     * @return the color the sensor sees 
     */
    private int getColor(){
        blueValue = getBlueValue();
        redValue = getRedValue();
        totalValue = blueValue + redValue;
//        System.out.println("BLUE: " + blueValue + " RED: " + redValue);//DON'T HAVE A LOGGER
        if(blueValue > redValue && totalValue < RED_THRESHOLD){
            return BLUE_COLOR;
        }else if(blueValue < redValue && totalValue < BLUE_THRESHOLD){
            return RED_COLOR;
        }else if(totalValue > WHITE_THRESHOLD){
            return WHITE_COLOR;
        }else if(totalValue < CARPET_THRESHOLD){
            return CARPET_COLOR;
        }else{
            return UNKNOWN_COLOR;
        }
    }
    
    /**
     * @param color - a constant
     * @return true if the color given is the same color seen by the sensor
     */
    public boolean isLineColor(int color){
        return(color == getColor());
    }
}
