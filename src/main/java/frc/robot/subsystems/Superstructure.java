package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Robot;

public class Superstructure implements Subsystem {

    private Compressor compressor;
    
    private List actions;

    //private Elevator elevator = Robot.elevator;
   // private CargoIntake cargoIntake = Robot.cargoIntake;
   // private HatchMechanism hatch = Robot.hatch;


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