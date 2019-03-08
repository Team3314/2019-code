package frc.robot.statemachines;

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

    public enum GamePieceState {
        WAITING,
        ALIGNING,
        DRIVING,
        GRABBING_HATCH,
        PLACING_HATCH,
        PLACING_BALL,
        BACKUP_HATCH,
        BACKUP_BALL,
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


    private GamePieceState currentState = GamePieceState.WAITING, nextState;
    private GamePieceStateMachineMode mode = GamePieceStateMachineMode.HATCH_LEVEL1;
    private Drive drive = Robot.drive;
    private Elevator elevator = Robot.elevator;
    private CargoIntake cargoIntake = Robot.cargoIntake;
    private HatchMechanism hatch = Robot.hatch;

    private Timer timer = new Timer();

    private boolean request;
    private boolean lastRequest;

    private boolean hasCollided = false;

    private double driveSpeed = .6, visionOffset = 0;


    private int desiredElevatorHeight = 0;

    
    public void update() {
        if(!request && lastRequest) {
            currentState = GamePieceState.WAITING;
            timer.stop();
            timer.reset();

        }
        switch(currentState) {
            case WAITING:
                if(request) {
                    hasCollided = false;
                    drive.setDriveMode(DriveMode.VISION_CONTROL);
                    drive.setVisionOffset(visionOffset);
                    elevator.setElevatorState(ElevatorControlMode.MOTION_MAGIC);
                    currentState = GamePieceState.ALIGNING;
                }
                else {  
                    drive.setVisionOffset(0);
                    driveSpeed = .6;
                }
                break;
            case ALIGNING:
                if(drive.gyroInPosition()) {
                    drive.setVisionOffset(0);
                    currentState = GamePieceState.DRIVING;
                }
                break;
            case DRIVING:
                drive.set(driveSpeed, driveSpeed);
                switch(mode) { 
                    case HATCH_LEVEL1:
                        desiredElevatorHeight = Constants.kElevatorHatchLevel1;
                        nextState = GamePieceState.PLACING_HATCH;
                        break;
                    case HATCH_LEVEL2:
                        desiredElevatorHeight = Constants.kElevatorHatchLevel2;
                        nextState = GamePieceState.PLACING_HATCH;
                        break;
                    case HATCH_LEVEL3:
                        desiredElevatorHeight = Constants.kElevatorHatchLevel3;
                        nextState = GamePieceState.PLACING_HATCH;
                        break;
                    case BALL_LEVEL1:
                        desiredElevatorHeight = Constants.kElevatorBallLevel1;
                        nextState = GamePieceState.PLACING_BALL;
                        break;
                    case BALL_LEVEL2:
                        desiredElevatorHeight = Constants.kElevatorBallLevel2;
                        nextState = GamePieceState.PLACING_BALL;
                        break;
                    case BALL_LEVEL3:
                        desiredElevatorHeight = Constants.kElevatorBallLevel3;
                        nextState = GamePieceState.PLACING_BALL;
                        break;
                    case HATCH_PICKUP:
                        desiredElevatorHeight = Constants.kElevatorHatchPickup;
                        nextState = GamePieceState.GRABBING_HATCH;
                        break;
                }
                if(drive.getDistanceToTarget() <= 48) {
                    if(mode == GamePieceStateMachineMode.HATCH_PICKUP) {
                        hatch.setGripperDown(true);
                    }/*
                    if(drive.getDistanceSensor()) {
                        drive.setDriveMode(DriveMode.GYROLOCK);
                        drive.setGyroDriveDistance(drive.getDistanceToTarget());
                        if(drive.collision()) {
                            hasCollided = true;
                        }
                    }*/
                    elevator.set(desiredElevatorHeight);   
                    
                    if(drive.getDistanceSensor() && elevator.inPosition()) {
                        currentState = nextState;
                        drive.set(0,0);
                    }

                }
                break;
            case GRABBING_HATCH:
                hatch.setIntakeRequest(true);
                if(hatch.isDone()) {
                    hatch.setIntakeRequest(false);
                    currentState = GamePieceState.BACKUP_HATCH;
                    drive.resetDriveEncoders();
                }
                break;
            case PLACING_BALL:
                cargoIntake.setIntakeState(IntakeState.PLACE);
                if(!cargoIntake.getCargoCarriageSensor()) {
                    drive.resetDriveEncoders();
                    currentState = GamePieceState.BACKUP_BALL;
                }
                break;
            case PLACING_HATCH:
                hatch.setPlaceRequest(true);
                if(hatch.isDone()) {
                    hatch.setPlaceRequest(false);
                    currentState = GamePieceState.BACKUP_HATCH;
                    drive.resetDriveEncoders();
                }
                break;
            case BACKUP_HATCH:
                drive.setDriveMode(DriveMode.GYROLOCK);
                drive.set(-.25, -.25);
                if(drive.getAverageRioPosition() <= -12) {
                    hatch.setRetractRequest(true);
                    drive.set(0, 0);
                    currentState = GamePieceState.RETRACT;
                }
                break;
            case BACKUP_BALL:
                drive.setDriveMode(DriveMode.GYROLOCK);
                elevator.set(Constants.kElevatorHatchPickup);
                drive.set(-.25, -.25);
                if(drive.getAverageRioPosition() <= -12) {
                    hatch.setRetractRequest(true);
                    drive.set(0, 0);
                    currentState = GamePieceState.WAITING;
                }
                break;
            case RETRACT:
                elevator.set(Constants.kElevatorHatchPickup);
                hatch.setRetractRequest(false);
                currentState = GamePieceState.DONE;
                break;
            case DONE:
                break;
        }
        lastRequest = request;
    }

    
    public void outputToSmartDashboard() {
    }

    public void debug() {
        SmartDashboard.putString("Game Piece State Machine State", currentState.toString());
        SmartDashboard.putString("Game Piece State Machine Mode", mode.toString());
        SmartDashboard.putNumber("Game Piece State Machine Elevator Height", desiredElevatorHeight);
    }

    
    public boolean isDone() {
        return currentState == GamePieceState.DONE;
    }

    public GamePieceState getCurrentState() {
        return currentState;
    }

    public void setRequest(boolean request) {
        this.request = request;
    }

    public void setMode(GamePieceStateMachineMode mode) {
        this.mode = mode;
    }

    public void setDriveSpeed(double speed) {
        driveSpeed = speed;
    }
    public void setVisionOffset(double offset) {
        visionOffset = offset;
    }
    public boolean isPlacing() {
        return currentState == GamePieceState.PLACING_BALL || currentState == GamePieceState.PLACING_HATCH || currentState == GamePieceState.GRABBING_HATCH;
    }
}
