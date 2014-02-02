package org.gosparx.subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import org.gosparx.util.Logger;

/**
 * The most basic version of any subsystem.  This class is used to have a 
 * framework for all subsystems.  It will be used to start all subsystems and 
 * keep them running even if exceptions are thrown.
 *
 * @author Justin Bassett (Bassett.JustinT@gmail.com)
 */
public abstract class GenericSubsystem extends Thread {
    
    /**
     * An Instance of DriverStation
     */
    protected DriverStation ds;
    
    /**
     * A logger. This is used for logging purposes
     */
    protected Logger log;
    protected DriverStation ds;
    
    /**
     * The time in seconds between logging
     */
    protected final double LOG_EVERY = 5.0;
    
    /**
     * The last time data was logged
     */
    protected double lastLogTime;
    
    /**
     * This creates a generic subsystem.
     *
     * @param nameOfSubsystem A debugging name for the subsystem. Use a constant
     * from {@link Logger} constant, ie. {@link Logger#SUB_DRIVES}.
     * @param threadPriority The {@link java.lang.Thread}'s priority.
     */
    public GenericSubsystem(String nameOfSubsystem, int threadPriority){
        super(nameOfSubsystem);
        ds = DriverStation.getInstance();
        this.setPriority(threadPriority);
        ds = DriverStation.getInstance();
        if(!nameOfSubsystem.equals("LogWriter")){
            log = new Logger(nameOfSubsystem);   
        }
    }
    
    /**
     * This method is in charge of keeping executeAuto and executeTele running.
     * It must recall the correct method if it crashes.
     */
    public void run(){
        while (true) {
            try {
                if(!ds.isTest()){
                    execute();   
                }
                Thread.sleep(10);
            } catch (Throwable e) {
                log.logError("Uncaught Exception: " + e.getMessage());
                e.printStackTrace();
            }
        }
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
     * is in charge of making sure to follow the mode rules!  To get the current
     * mode use {@link DriverStation#isEnabled() isEnabled()} or 
     * {@link DriverStation#isAutonomous() isAutonomous()}
     * 
     * @throws Exception
     */
    public abstract void execute() throws Exception;
    /**
     * Determines if the last autonomous command is finished
     * @return true - the last issued command to the subsystem is done
     *         false - the last issued commands to the subsystem is not done
     */
    public boolean isLastCommandDone(){
        return true;
    }
    
    public  abstract void liveWindow();
    
    public abstract void liveWindow();
    
}
