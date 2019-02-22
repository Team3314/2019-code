package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class GamePieceStateMachine extends StateMachine {

    public enum State {
        WAITING,
        DRIVING,
        GRABBING_HATCH,
        PLACING_HATCH,
        PLACING_BALL,
        DONE
    }

    private State currentState = State.WAITING, nextState;
    private Drive drive = Robot.drive;
    private Elevator elevator = Robot.elevator;
    private CargoIntake cargoIntake = Robot.cargoIntake;
    private HatchMechanism hatch = Robot.hatch;
    private HatchIntakeStateMachine hatchIntakeStateMachine = Robot.hatchIntakeStateMachine;
    private HatchPlaceStateMachine hatchPlaceStateMachine = Robot.hatchPlaceStateMachine;
    private TrackingStateMachine tracking = Robot.trackingStateMacahine;
    private Camera camera = Robot.camera;
    private HumanInput HI = Robot.HI;

    private int desiredElevatorHeight = 0;

    @Override
    public void update() {
        if(!request) {
            currentState = State.WAITING;
        }
        switch(currentState) {
            case WAITING:
                if(request) {
                    tracking.setRequest(true);
                    currentState = State.DRIVING;
                }
                break;
            case DRIVING:
                if(HI.getElevatorLevel1()) {
                    if(cargoIntake.getCargoInCarriage()) {
                        nextState = State.PLACING_BALL;
                        desiredElevatorHeight = Constants.kElevatorBallLevel1;
                    }
                    else {
                        nextState = State.PLACING_HATCH;
                        desiredElevatorHeight = Constants.kElevatorHatchLevel1;
                    }
                }
                else if (HI.getElevatorLevel2()) {
                    if(cargoIntake.getCargoInCarriage()) {
                        nextState = State.PLACING_BALL;
                        desiredElevatorHeight = Constants.kElevatorBallLevel2;
                    }
                    else {
                        nextState = State.PLACING_HATCH;
                        desiredElevatorHeight = Constants.kElevatorHatchLevel2;
                    }
                }
                else if(HI.getElevatorLevel3()) {
                    if(cargoIntake.getCargoInCarriage()) {
                        nextState = State.PLACING_BALL;
                        desiredElevatorHeight = Constants.kElevatorBallLevel3;
                    }
                    else {
                        nextState = State.PLACING_HATCH;
                        desiredElevatorHeight = Constants.kElevatorHatchLevel3;
                    }
                }
                else if(HI.getElevatorPickup()) {
                    nextState = State.GRABBING_HATCH;
                    desiredElevatorHeight = Constants.kElevatorHatchPickup;
                }
                if(camera.getRawDistance() <= 36) {
                    elevator.set(desiredElevatorHeight);
                }
                if(tracking.isDone() && elevator.inPosition()) {
                    tracking.setRequest(false);
                    currentState = nextState;
                }
                break;
            case GRABBING_HATCH:
                hatchIntakeStateMachine.setRequest(true);
                if(hatchIntakeStateMachine.isDone()) {
                    hatchIntakeStateMachine.setRequest(false);
                    currentState = State.DONE;
                }
                break;
            case PLACING_BALL:
                cargoIntake.setIntakeState(IntakeState.PLACE);
                if(!cargoIntake.getCargoInCarriage()) {
                    currentState = State.DONE;
                }
                break;
            case PLACING_HATCH:
                hatchPlaceStateMachine.setRequest(true);
                if(hatchPlaceStateMachine.isDone()) {
                    currentState = State.DONE;
                }
                break;
            case DONE:  
                break;
        }
        lastRequest = request;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Game Piece State Machine State", currentState.toString());
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }

}