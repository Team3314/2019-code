package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

public class Superstructure implements Subsystem {

    private Compressor compressor;

    public Superstructure(Compressor compressor) {
        this.compressor = compressor;

    }

    public void update(){


    }

    public void outputToSmartDashboard(){
    }   

    public void resetSensors(){

    }

    public void startCompressor() {
        compressor.start();
        compressor.setClosedLoopControl(true);
    }
    
    public void stopCompressor() {
        compressor.stop();
        compressor.setClosedLoopControl(false);
    }

}