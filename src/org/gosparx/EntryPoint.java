/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.gosparx;

import edu.wpi.first.wpilibj.SimpleRobot;
import org.gosparx.subsystem.GenericSubsystem;
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
    
    /**
     * Robot-wide initialization code should go here. Users should override this 
     * method for default Robot-wide initialization which will be called when 
     * the robot is first powered on. Called exactly 1 time when the competition 
     * starts.
     */
    public void robotInit(){
        //TODO: Log init starting
        subsystems = new GenericSubsystem[0];
        
        // Place the subsytems here
        
        
        for (int i = 0; i < subsystems.length; i++) {
            subsystems[i].init();
            subsystems[i].start();
        }
        Logger logger = new Logger("Test");
        logger.logMessage("Test");
        logger.close();
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        //TODO: Log swap to autonomous
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        //TODO: Log swap to operator control
    }
    
    /**
     * Disabled should go here. Users should overload this method to run code 
     * that should run while the field is disabled. Called once each time the 
     * robot enters the disabled state.
     */
    public void disabled(){
        //TODO: Log swap to disabled
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}
