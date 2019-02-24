package frc.robot.statemachines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class GamePieceStateMachine {

    public enum State {
        WAITING,
        DRIVING,
        GRABBING_HATCH,
        PLACING_HATCH,
        PLACING_BALL,
        BACKUP,
        DONE
    }

    public enum GamePieceStateMachineMode {
        HATCH_LEVEL1,
        HATCH_LEVEL2,
        HATCH_LEVEL3,
        BALL_LEVEL1,
        BALL_LEVEL2,
        BALL_LEVEL3,
        HATCH_PICKUP
    }
    private State currentState = State.WAITING, nextState;
    private GamePieceStateMachineMode mode = GamePieceStateMachineMode.HATCH_LEVEL1;
    private Drive drive = Robot.drive;
    private Elevator elevator = Robot.elevator;
    private CargoIntake cargoIntake = Robot.cargoIntake;
    private HatchMechanism hatch = Robot.hatch;
    private TrackingStateMachine tracking = Robot.trackingStateMacahine;
    private Camera camera = Robot.camera;
    private DriverStation ds = DriverStation.getInstance();

    private Timer timer = new Timer();

    private boolean request;
    private boolean lastRequest;


    private int desiredElevatorHeight = 0;

    
    public void update() {
        if(!request && lastRequest) {
            currentState = State.WAITING;
            timer.stop();
            timer.reset();

        }
        switch(currentState) {
            case WAITING:
                desiredElevatorHeight = 0;
                if(request) {
                    tracking.setTrackingRequest(true);
                    currentState = State.DRIVING;
                }
                break;
            case DRIVING:
                switch(mode) { 
                    case HATCH_LEVEL1:
                        desiredElevatorHeight = Constants.kElevatorHatchLevel1;
                        nextState = State.PLACING_HATCH;
                        break;
                    case HATCH_LEVEL2:
                        desiredElevatorHeight = Constants.kElevatorHatchLevel2;
                        nextState = State.PLACING_HATCH;
                        break;
                    case HATCH_LEVEL3:
                        desiredElevatorHeight = Constants.kElevatorHatchLevel3;
                        nextState = State.PLACING_HATCH;
                        break;
                    case BALL_LEVEL1:
                        desiredElevatorHeight = Constants.kElevatorBallLevel1;
                        nextState = State.PLACING_BALL;
                        break;
                    case BALL_LEVEL2:
                        desiredElevatorHeight = Constants.kElevatorBallLevel2;
                        nextState = State.PLACING_BALL;
                        break;
                    case BALL_LEVEL3:
                        desiredElevatorHeight = Constants.kElevatorBallLevel3;
                        nextState = State.PLACING_BALL;
                        break;
                    case HATCH_PICKUP:
                        desiredElevatorHeight = Constants.kElevatorHatchPickup;
                        break;
                }
                if(cargoIntake.getCargoInCarriage()) {
                    nextState = State.PLACING_BALL;
                }
                else { 
                    nextState = State.PLACING_HATCH;
                }
                if(camera.getRawDistance() <= 72) {
                    elevator.set(desiredElevatorHeight);
                }
                if(tracking.isDone() && elevator.inPosition()) {
                    tracking.setTrackingRequest(false);
                    currentState = nextState;
                }
                break;
            case GRABBING_HATCH:
                hatch.setIntakeRequest(true);
                if(hatch.isDone()) {
                    hatch.setIntakeRequest(false);
                    currentState = State.BACKUP;
                    timer.start();
                }
                break;
            case PLACING_BALL:
                cargoIntake.setIntakeState(IntakeState.PLACE);
                if(!cargoIntake.getCargoInCarriage()) {
                    currentState = State.BACKUP;
                    timer.start();
                }
                break;
            case PLACING_HATCH:
                hatch.setPlaceRequest(true);
                if(hatch.isDone()) {
                    hatch.setPlaceRequest(false);
                    currentState = State.BACKUP;
                    timer.start();
                }
                break;
            case BACKUP:
                elevator.set(Constants.kElevatorHatchPickup);
                drive.set(-.25, -.25);
                if(timer.get() >= .5) {
                    drive.set(0, 0);
                    currentState = State.DONE;
                }
                break;
            case DONE:
                break;
        }
        lastRequest = request;
    }

    
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Game Piece State Machine State", currentState.toString());
    }

    
    public boolean isDone() {
        return currentState == State.DONE;
    }

    public void setRequest(boolean request) {
        this.request = request;
    }

    public void setMode(GamePieceStateMachineMode mode) {
        this.mode = mode;
    }
}