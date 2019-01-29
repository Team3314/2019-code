package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Robot;

public class Superstructure implements Subsystem {

    private enum State {
        HOLDING,
        PICKUP_CARGO,
        PICKUP_HATCH,
        PLACE_CARGO,
        PLACE_HATCH
    }

    private Compressor compressor;

    private State wantedState;

    private Elevator elevator = Robot.elevator;
    private CargoIntake cargoIntake = Robot.cargoIntake;
    private HatchMechanism hatch = Robot.hatch;


    public Superstructure(Compressor compressor) {
        this.compressor = compressor;
    }

    public void update(){
        switch(wantedState) {
            case HOLDING:
                break;
            case PICKUP_CARGO:
                elevator.setPo
                break;
            case PICKUP_HATCH:
                break;
            case PLACE_CARGO:
                break;
            case PLACE_HATCH:
                break;
        }

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