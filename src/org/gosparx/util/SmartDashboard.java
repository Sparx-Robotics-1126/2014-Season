/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package org.gosparx.util;

import edu.wpi.first.wpilibj.Timer;
import org.gosparx.subsystem.GenericSubsystem;

/**
 *
 * @author Connor
 */
public class SmartDashboard extends GenericSubsystem{

    private edu.wpi.first.wpilibj.smartdashboard.SmartDashboard smart;
    private double time = 0;
    private double startingTime = 0;
    private double currentTime = 0;
    
    public SmartDashboard(){
        super("Smart", GenericSubsystem.MIN_PRIORITY);
    }
    
    public void init() {
        smart = new edu.wpi.first.wpilibj.smartdashboard.SmartDashboard();
    }

    public void execute() throws Exception {
        startingTime = Timer.getFPGATimestamp();
        while(ds.isEnabled()){
            startTimer();
        }
    }
    
    private void startTimer(){
        currentTime = Timer.getFPGATimestamp();
        time = currentTime - startingTime;
        smart.putNumber("Timer", (int)time);
        System.out.println(time);
    }
    
    public void isInRange(boolean inRange){
        smart.putBoolean("In Shooting Range", inRange);
    }
    
}
