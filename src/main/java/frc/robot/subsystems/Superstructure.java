package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure implements Subsystem {

    public enum GamePieceState {
        HAS_BALL,
        HAS_HATCH,
        HAS_NONE
    }

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