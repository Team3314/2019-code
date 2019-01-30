package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Robot;

public class Superstructure implements Subsystem {

    private Compressor compressor;

    private Elevator elevator = Robot.elevator;
    private CargoIntake cargoIntake = Robot.cargoIntake;
    private HatchMechanism hatch = Robot.hatch;


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
    }
    
    public void stopCompressor() {
        compressor.stop();
    }

}