package org.gosparx;

import org.gosparx.subsystem.Drives;
import org.gosparx.subsystem.Vision;


public class Autonomous {
    private Drives drives;
    private Vision vision;
    
    private Autonomous(){
//        drives = Drives.getInstance();
        drives = new Drives();//getInstance hasen't been implemented
        vision = Vision.getInstance();
        
    }
}
