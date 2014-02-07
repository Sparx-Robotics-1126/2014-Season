/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.gosparx;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.gosparx.subsystem.Acquisitions;
import org.gosparx.subsystem.Controls;
import org.gosparx.subsystem.Drives;
import org.gosparx.subsystem.GenericSubsystem;
import org.gosparx.subsystem.Shooter;
import org.gosparx.subsystem.Vision;
import org.gosparx.util.LogWriter;
import org.gosparx.util.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class EntryPoint extends SimpleRobot {
    /**
     * The list of all the subsystems.
     */
    private GenericSubsystem[] subsystems;
    private Logger logger;
    private Autonomous auto;
    private Vision vision;
    
    /**
     * Robot-wide initialization code should go here. Users should override this 
     * method for default Robot-wide initialization which will be called when 
     * the robot is first powered on. Called exactly 1 time when the competition 
     * starts.
     */
    public void robotInit(){
        //TODO: Log init starting
        subsystems = new GenericSubsystem[3];
        subsystems[0] = LogWriter.getInstance();
//        subsystems[1] = Drives.getInstance();
        subsystems[1] = Controls.getInstance();
        subsystems[2] = Autonomous.getInstance();
//        subsystems[3] = Acquisitions.getInstance();
//        subsystems[4] = Shooter.getInstance();
//        subsystems[4] = Vision.getInstance();
        logger = new Logger("Robot State");
        auto = Autonomous.getInstance();
        
        for (int i = 0; i < subsystems.length; i++) {
            subsystems[i].init();
            subsystems[i].liveWindow();
            subsystems[i].start();
        }
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        logger.logMessage("Switched to Autonomous");
        auto.runAuto(true);
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        logger.logMessage("Switched to Teleop");
        auto.runAuto(false);
    }
    
    /**
     * Disabled should go here. Users should overload this method to run code 
     * that should run while the field is disabled. Called once each time the 
     * robot enters the disabled state.
     */
    public void disabled(){
        logger.logMessage("Switched to Disabled");
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
            while(isTest() && isEnabled()){
                LiveWindow.run();
                Timer.delay(0.1);
            }
    }
    
}
