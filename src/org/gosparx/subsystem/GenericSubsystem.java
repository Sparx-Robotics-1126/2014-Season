package org.gosparx.subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    
    /**
     * The time in seconds between logging
     */
    protected final double LOG_EVERY = 5.0;
    
    /**
     * The last time data was logged
     */
    protected double lastLogTime;
    
    /**
     * The number of times stored for the average times.
     */
    private final static int RING_LENGTH = 10;
    
    /**
     * The ring buffer for average run times.
     */ 
    private double[] ringBuffer = new double[RING_LENGTH];
    
    /**
     * The starting location for the ring buffer.
     */ 
    private int ringLoc = 0;
    
    /**
     * The start time for the current loop. Used in average runtime calculations.
     */ 
    private double startTime;
    
    
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
        init();
        liveWindow();
        while (true) {
            try {
                if(!ds.isTest()){
                    startTime = Timer.getFPGATimestamp();
                    execute();
                    addRunTime(Timer.getFPGATimestamp() - startTime);
                    if(Timer.getFPGATimestamp() - lastLogTime >= LOG_EVERY){
                        logInfo();
                        lastLogTime = Timer.getFPGATimestamp();
                    }
                }
                Thread.sleep(sleepTime());
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
    
    /**
     * Add the runTime to the ring buffer and increment the ring buffer.
     * @param runTime - The last runtime in seconds.
     */ 
    protected void addRunTime(double runTime){
        ringBuffer[ringLoc] = runTime;
        ringLoc ++;
        ringLoc %= RING_LENGTH;
    }
    
    /**
     * Returns the average runtime of all of the runtimes in the ring buffer.
     * @return - the average runtime off all of the runtimes in the ring buffer.
     */ 
    protected double getAverageRuntime(){
        double total = 0.0;
        for(int i = 0; i < RING_LENGTH; i++){
            total += ringBuffer[i];
        }
        return total/RING_LENGTH;
    }    
    
    /**
     * @return The time in ms to sleep for after each loop
     */ 
    public abstract int sleepTime();
    
    /**
     * Log all info about the subsystem.
     */ 
    public abstract void logInfo();
    
}
