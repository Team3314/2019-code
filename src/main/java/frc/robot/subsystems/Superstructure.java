package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

public class Superstructure implements Subsystem {

    private Compressor compressor;
    
    public Superstructure() {
        compressor = new Compressor();
        compressor.setClosedLoopControl(true);
    }

    public void update(){

    }

    public void outputToSmartDashboard(){

    }   

    public void resetSensors(){

    }

    public void startCompressor() {
        compressor.start();
    }
    
    public void stopCompressor() {
        compressor.stop();
    }

}