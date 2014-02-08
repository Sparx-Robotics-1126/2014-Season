package org.gosparx;

import edu.wpi.first.wpilibj.AnalogChannel;

/**
 * The purpose of this class is to have all the electrical configuration in one 
 * place.
 *
 * @author Justin Bassett (Bassett.JustinT@gmail.com)
 */
public class IO {
    
    // Digital Side Card Slots
    public static final int DEFAULT_SLOT                = 1;
    
    // Drives PWM Slots
    public static final int LEFT_FRONT_DRIVES_PWM       = 4;
    public static final int LEFT_REAR_DRIVES_PWM        = 5;
    public static final int RIGHT_FRONT_DRIVES_PWM      = 7;
    public static final int RIGHT_REAR_DRIVES_PWM       = 8;
    
    // Drives Encoder Slots
    public static final int LEFT_DRIVES_ENCODER_CHAN_1  = 1;
    public static final int LEFT_DRIVES_ENCODER_CHAN_2  = 2;
    public static final int RIGHT_DRIVES_ENCODER_CHAN_1 = 3;
    public static final int RIGHT_DRIVES_ENCODER_CHAN_2 = 4;
    
    // Drives Pnumatic Slots
    public static final int SHIFT_CHAN                  = 1;
    
    //Drives Gyro slot
    public static final AnalogChannel GYRO_ANALOG       = new AnalogChannel(1);
    
    //Drives Booleans
    public static final boolean DRIVES_TURN_LEFT        = true;
    public static final boolean DRIVES_TURN_RIGHT       = false;
   
    public static final int PRESSURE_SWITCH_CHAN        = 14;
    public static final int COMPRESSOR_RELAY_CHAN       = 1;
    
    // Controls IO Ports
        
    public static final int LEFT_DRIVER_JOY_PORT        = 1;
    
    public static final int RIGHT_DRIVER_JOY_PORT       = 2;
    
    public static final int OPER_JOY_PORT               = 3;
    
    //Analog Ports
    public static final int DEFAULT_ANALOG_MODULE       = 1;
    public static final int AUTOSWITCH_CHANNEL          = 2;
    
    //Shooter IO
    public static final int WINCH_POT_CHAN              = 3;
    
    public static final int CAN_ADRESS_PIVOT            = 4;//4
    
    public static final int CAN_ADRESS_WINCH            = 2;//2
    
    public static final int CAN_ADRESS_ACQ              = 3;
    
    public static final int PIVOT_ENCODER_CHAN_1        = 5;
    
    public static final int PIVOT_ENCODER_CHAN_2        = 6;
    
    public static final int ACQ_TOGGLE_CHAN             = 3;
    
    public static final int ACQ_SWITCH_CHAN             = 12;
    
    public static final int LATCH_SWITCH_CHAN           = 11;
    
    public static final int SHOOTER_ACQ_MODE_CHAN       = 9;
    
    public static final int SHOOTER_SAFE_MODE_CHAN      = 10;
    
    public static final int LATCH_CHAN                  = 4;
    
    public static final int KEEP_IN_FRAME_CHAN          = 2;
}
