package frc.robot.statemachines;

import frc.robot.Constants;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;

public class GamePieceStateMachine extends StateMachine {

    public enum State {
        WAITING,
        DRIVING,
        GRABBING_HATCH,
        PLACING_HATCH,
        PLACING_BALL
    }

    private State currentState = State.WAITING;
    private Drive drive = Robot.drive;
    private Elevator elevator = Robot.elevator;
    private CargoIntake cargoIntake = Robot.cargoIntake;
    private HatchMechanism hatch = Robot.hatch;
    private TrackingStateMachine tracking = Robot.trackingStateMacahine;
    private Camera camera = Robot.camera;
    private HumanInput HI = Robot.HI;

    private int desiredElevatorHeight = 0;

    @Override
    public void update() {
        switch(currentState) {
            case WAITING:
                if(request) {
                    tracking.setRequest(true);
                    currentState = State.DRIVING;
                }
                break;
            case DRIVING:
                if(HI.getElevatorLevel1()) {
                    if(cargoIntake.getCargoInCarriage())
                        desiredElevatorHeight = Constants.kElevatorBallLevel1;
                    else
                        desiredElevatorHeight = Constants.kElevatorHatchLevel1;
                }
                else if (HI.getElevatorLevel2()) {
                    if(cargoIntake.getCargoInCarriage())
                        desiredElevatorHeight = Constants.kElevatorBallLevel2;
                    else
                        desiredElevatorHeight = Constants.kElevatorHatchLevel2;
                }
                else if(HI.getElevatorLevel3()) {
                    if(cargoIntake.getCargoInCarriage())
                        desiredElevatorHeight = Constants.kElevatorBallLevel3;
                    else
                        desiredElevatorHeight = Constants.kElevatorHatchLevel3;
                }
                if(camera.getRawDistance() <= 36) {
                    elevator.set(desiredElevatorHeight);
                }
                break;
            case GRABBING_HATCH:
                if(tracking.isDone()) {
                    
                }
                break;
            case PLACING_BALL:
                if(tracking.isDone()) {

                }
                break;
            case PLACING_HATCH:
                if(tracking.isDone()) {

                }
                break;
        }
        lastRequest = request;
    }

    @Override
    public void outputToSmartDashboard() {

    }

}