package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

public class Superstructure implements Subsystem {

    private Compressor compressor = new Compressor();
    
    private static Superstructure mInstance = new Superstructure();

    /**
     * @return the mInstance
     */
    public static Superstructure getInstance() {
        return mInstance;
    }

    private Superstructure() {
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