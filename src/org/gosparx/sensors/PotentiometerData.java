/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.gosparx.sensors;

import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * @author Alex
 * A class for interpreting the data we receive from a Potentiometer
 */
public class PotentiometerData {
    
    /**
     * The Potentiometer to get the data from.
     */ 
    private Potentiometer pot;
    
    /**
     * The inches that the potentiometer travels per volt changed.
     */ 
    private double inchesPerVolt;
    
    /**
     * The "zero" point in volts that we will use our zero for calculations.
     */
    private double zeroPointVolts;
    
    /**
     * Creates a new PotentiometerData.
     * @param pot - The Potentiometer to get data from
     * @param inchesPerVolt - the inches that the potentiometer moves per volt 
     * of change
     */
    public PotentiometerData(Potentiometer pot, double inchesPerVolt){
        this.pot = pot;
        this.inchesPerVolt = inchesPerVolt;
        this.reset();
    }
    
    /**
     * Set the "zero" point of the potentiometer to the current voltage.
     */
    public void reset(){
        zeroPointVolts = pot.get();
    }
    
    /**
     * @return the inches from the "zero" point the potentiometer is.
     */ 
    public double getInches(){
        return ((pot.get() - zeroPointVolts) * inchesPerVolt);
    }
}
