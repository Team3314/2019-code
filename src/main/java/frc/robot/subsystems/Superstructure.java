package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.actions.*;

public class Superstructure implements Subsystem {

    public enum GamePieceState {
        HAS_BALL,
        HAS_HATCH,
        HAS_NONE
    }

    private Compressor compressor;

    private Elevator elevator;
    private CargoIntake cargoIntake;
    private HatchMechanism hatch;
    
    private Deque<Action> actions = new ArrayDeque<Action>();

    boolean autoGamePiece = false;

    public Superstructure(Compressor compressor) {
        this.compressor = compressor;
        elevator = Robot.elevator;
        cargoIntake = Robot.cargoIntake;
        hatch = Robot.hatch;

    }

    public void update(){
        if(!isQueueDone()) {
            if(actions.getFirst().isDone()) {
                actions.poll();
                actions.getFirst().init();
            }
                actions.getFirst().update();
        }
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putBoolean("isEmpty", isQueueDone());
        SmartDashboard.putString("Action state", actions.getFirst().getState().toString());
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
        compressor.setClosedLoopControl(true);
    }
    
    public void stopCompressor() {
        compressor.stop();
        compressor.setClosedLoopControl(false);
    }
    public void setAutoGamePiece(int level) {
        if(cargoIntake.getHasCargo()) {
            actions.add(new CargoPlaceRocket());
        }
        else if(hatch.getHasHatch()) {
            actions.add(new HatchPlaceRocket(level));
        }
        else {
            actions.add(new HatchPickup());
        }
    }

}