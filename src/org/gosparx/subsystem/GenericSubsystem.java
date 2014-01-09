package org.gosparx.subsystem;

import com.sun.squawk.util.NotImplementedYetException;

/**
 * The most basic version of any subsystem.  This class is used to have a 
 * framework for all subsystems.  It will be used to start all subsystems and 
 * keep them running even if exceptions are thrown.
 *
 * @author Justin Bassett (Bassett.JustinT@gmail.com)
 */
public abstract class GenericSubsystem extends Thread {
    
    /**
     * This mode is used when the robot is in auto mode.
     */
    public static final int MODE_AUTO = 0;
    
    /**
     * This mode is used when the robot is in teleop mode.
     */
    public static final int MODE_TELE = 1;
    
    /**
     * This mode is used when the robot is in a disabled mode.
     */
    public static final int MODE_DISABLED = 2;
    
    /**
     * This is the name of the subsystem, used for logs.
     */
    protected String nameOfSubsystem;
    
    /**
     * The mode of the robot.
     */
    protected int mode;
    
    /**
     * This creates a generic subsystem.
     *
     * @param nameOfSubsystem A debugging name for the subsystem.
     * @param threadPriority The {@link java.lang.Thread}'s priority.
     */
    public GenericSubsystem(String nameOfSubsystem, int threadPriority){
        this.nameOfSubsystem = nameOfSubsystem;
        this.setPriority(threadPriority);
    }
    
    /**
     * Sets the robot mode based on the field or operator controls.
     *
     * @param mode either MODE_AUTO, MODE_TELE, MODE_DISABLED
     */
    public void setMode(int mode){
        switch(mode){
            case MODE_AUTO:
            case MODE_TELE:
            case MODE_DISABLED:
                this.mode = mode;
                break;
            default:
                throw new RuntimeException("Cannot set mode to something unknown!");
        }
        this.mode = mode;
    }
    
    /**
     * This method is in charge of keeping executeAuto and executeTele running.
     * It must recall the correct method if it crashes.
     */
    public void run(){
        throw new NotImplementedYetException();
        //TODO: Implement me!
    }
    
    /**
     * This method gets called once when the robot is powered on.  The purpose 
     * of this method is to init things that take longer than should be allowed 
     * in constructor.
     */
    public abstract void init();
    
    /**
     * This method is executed repeatedly while the robot is on.  The intent for
     * this method is to be auto restarted when things crash.  The implementer 
     * is in charge of making sure to follow the mode rules!
     */
    public abstract void execute();
    
}
