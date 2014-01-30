/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.gosparx.subsystem;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import org.gosparx.IO;
import org.gosparx.util.Logger;

/**
 * @author Alex
 */
public class Shooter extends GenericSubsystem{
    
    private CANJaguar winchMotor;
    
    private DigitalInput latchSwitch;
    
    private int shooterState;
    
    private Solenoid latch;
    
    private final double WINCH_SPEED = 1.0;
    
    private final boolean LATCH_ENGAGED = true;
    
    private final boolean LATCH_DISENGAGED = !LATCH_ENGAGED;
    
    private double wantedWinchSpeed;
    
    private double lastShotTime;
    
    private final double TIME_BETWEEN_SHOTS = .5;
    
    private Shooter(){
        super(Logger.SUB_SHOOTER, Thread.NORM_PRIORITY);
    }

    public void init() {
        try {
            winchMotor = new CANJaguar(IO.CAN_ADRESS_WINCH);
        } catch (CANTimeoutException ex) {
            log.logError("CANBus timeout in Shooter init()");
        }
        latchSwitch = new DigitalInput(IO.LATCH_SWITCH_CHAN);
        latch = new Solenoid(IO.LATCH_CHAN);
        latch.set(LATCH_ENGAGED);
        shooterState = State.STANDBY;
    }

    public void execute() throws Exception {
        wantedWinchSpeed = 0;
        switch(shooterState){
            case State.SHOOT:
                latch.set(LATCH_DISENGAGED);
                lastShotTime = Timer.getFPGATimestamp();
                if(Timer.getFPGATimestamp() - lastLogTime >= TIME_BETWEEN_SHOTS){
                    shooterState = State.RETRACT;
                }
                break;
            case State.RETRACT:
                wantedWinchSpeed = WINCH_SPEED;
                if(latchSwitch.get()){
                    wantedWinchSpeed = 0;
                    shooterState = State.STANDBY;
                }
                break;
            case State.STANDBY:
                wantedWinchSpeed = 0;
                break;
            default:
                log.logError("Unknow Shooter state: " + shooterState);
                break;
        }
        winchMotor.setX(wantedWinchSpeed);
    }
    
    public void shoot(){
        shooterState = State.SHOOT;
    }
    
    private class State{
        public static final int SHOOT = 1;
        public static final int RETRACT = 2;
        public static final int STANDBY = 3;
    }
   
}
