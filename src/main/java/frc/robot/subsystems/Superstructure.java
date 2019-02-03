package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.actions.Action;

public class Superstructure implements Subsystem {

    public enum State {
        MANUAL,
        AUTOMATIC
    }

    private Compressor compressor;
    
    private List<Action> actions = new ArrayList<Action>();

    //private Elevator elevator = Robot.elevator;
   // private CargoIntake cargoIntake = Robot.cargoIntake;
   // private HatchMechanism hatch = Robot.hatch;


    public Superstructure(Compressor compressor) {
        this.compressor = compressor;
    }

    public void update(){
        if(!isQueueDone()) {
            if(actions.get(0).isDone()) {
                actions.remove(0);
            }
                actions.get(0).update();
        }
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putBoolean("isEmpty", isQueueDone());
    }   

    public void resetSensors(){

    }

    public void addAction(Action action) {
        actions.add(action);
    }

    public boolean isQueueDone() {
        return actions.isEmpty();
    }
    public void clearQueue() {
        actions.clear();
    }

    public void startCompressor() {
        compressor.start();
    }
    
    public void stopCompressor() {
        compressor.stop();
    }

}