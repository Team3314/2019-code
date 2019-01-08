package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

public class Superstructure implements Subsystem {

    private Compressor pcm1 = new Compressor();
    
    private static Superstructure mInstance = new Superstructure();

    /**
     * @return the mInstance
     */
    public static Superstructure getInstance() {
        return mInstance;
    }

    public void update(){

    }

    public void outputToSmartDashboard(){

    }   

    public void resetSensors(){

    }

}