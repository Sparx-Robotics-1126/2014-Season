package org.gosparx.subsystem;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The most basic version of any subsystem.  This class is used to have a 
 * framework for all subsystems.  It will be used to start all subsystems and 
 * keep them running even if exceptions are thrown.
 *
 * @author Justin Bassett (Bassett.JustinT@gmail.com)
 */
public abstract class GenericSubsystem extends Thread {
    /**
     * This is the name of the subsystem, used for logs.
     */
    protected DriverStation ds;
    protected String nameOfSubsystem;
    
    /**
     * This creates a generic subsystem.
     *
     * @param nameOfSubsystem A debugging name for the subsystem.
     * @param threadPriority The {@link java.lang.Thread}'s priority.
     */
    public GenericSubsystem(String nameOfSubsystem, int threadPriority){
        super(nameOfSubsystem);
        this.nameOfSubsystem = nameOfSubsystem;
        this.setPriority(threadPriority);
        ds = DriverStation.getInstance();
    }
    
    /**
     * This method is in charge of keeping executeAuto and executeTele running.
     * It must recall the correct method if it crashes.
     */
    public void run(){
        while (true) {
            try {
                execute();
                Thread.sleep(10);
            } catch (Throwable e) {
                // TODO: Log me!
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
    
}
