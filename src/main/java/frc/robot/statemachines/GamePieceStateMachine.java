package frc.robot.statemachines;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.CargoIntake.IntakeState;
import frc.robot.subsystems.Drive.DriveMode;
import frc.robot.subsystems.Elevator.ElevatorControlMode;

public class GamePieceStateMachine {

    public enum State {
        WAITING,
        ALIGNING,
        DRIVING,
        GRABBING_HATCH,
        PLACING_HATCH,
        PLACING_BALL,
        BACKUP,
        RETRACT,
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
    private DriverStation ds = DriverStation.getInstance();

    private Timer timer = new Timer();

    private boolean request;
    private boolean lastRequest;
    private boolean hasCollided = false;


    private int desiredElevatorHeight = 0;

    
    public void update() {
        if(!request && lastRequest) {
            currentState = State.WAITING;
            timer.stop();
            timer.reset();

        }
        switch(currentState) {
            case WAITING:   
                hasCollided = false;
                if(request) {
                    drive.setDriveMode(DriveMode.VISION_CONTROL);
                    elevator.setElevatorState(ElevatorControlMode.MOTION_MAGIC);
                    currentState = State.ALIGNING;
                }
                break;
            case ALIGNING:
                if(drive.gyroInPosition()) {
                    currentState = State.DRIVING;
                }
                break;
            case DRIVING:
                    drive.set(.5, .5);
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
                        nextState = State.GRABBING_HATCH;
                        break;
                }
                if(drive.collision()) {
                    hasCollided = true;
                }
                if(drive.getDistanceToTarget() <= 36) {
                    //if(mode == GamePieceStateMachineMode.HATCH_PICKUP)
                        //hatch.setGripperDown(true);
                    elevator.set(desiredElevatorHeight);
                    if(drive.getAtTarget() && elevator.inPosition()) {
                        currentState = nextState;
                    }

                }
                break;
            case GRABBING_HATCH:
                hatch.setIntakeRequest(true);
                if(hatch.isDone()) {
                    hatch.setIntakeRequest(false);
                    currentState = State.BACKUP;
                    drive.resetDriveEncoders();
                }
                break;
            case PLACING_BALL:
                cargoIntake.setIntakeState(IntakeState.PLACE);
                if(!cargoIntake.getCargoInCarriage()) {
                    drive.resetDriveEncoders();
                    currentState = State.BACKUP;
                }
                break;
            case PLACING_HATCH:
                hatch.setPlaceRequest(true);
                if(hatch.isDone()) {
                    hatch.setPlaceRequest(false);
                    currentState = State.BACKUP;
                    drive.resetDriveEncoders();
                }
                break;
            case BACKUP:
                drive.setDriveMode(DriveMode.GYROLOCK);
                drive.set(-.25, -.25);
                if(drive.getAverageRioPosition() <= -12) {
                    hatch.setRetractRequest(true);
                    drive.set(0, 0);
                    currentState = State.RETRACT;
                }
                break;
            case RETRACT:
                elevator.set(Constants.kElevatorHatchPickup);
                hatch.setRetractRequest(false);
                currentState = State.DONE;
                break;
            case DONE:
                break;
        }
        lastRequest = request;
    }

    
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Game Piece State Machine State", currentState.toString());
        SmartDashboard.putString("Game Piece State Machine Mode", mode.toString());
        SmartDashboard.putNumber("Game Piece State Machine Elevator Height", desiredElevatorHeight);
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