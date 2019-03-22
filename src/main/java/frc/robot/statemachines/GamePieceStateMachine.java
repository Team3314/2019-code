package frc.robot.statemachines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.HumanInput;
import frc.robot.subsystems.Drive.DriveMode;

public class GamePieceStateMachine {

    public enum GamePieceState {
        WAITING,
        GRABBING_HATCH,
        GRABBING_BALL,
        PLACING_HATCH,
        PLACING_BALL,
        BACKUP_HATCH,
        BACKUP_BALL,
        RETRACT,
        DONE
    }

    public enum GamePieceStateMachineMode {
        LEVEL1,
        LEVEL2,
        LEVEL3,
        CARGO_PICKUP,
        HATCH_PICKUP,
        CARGO_SHIP
    }


    private GamePieceState currentState = GamePieceState.WAITING, nextState;
    private GamePieceStateMachineMode mode = GamePieceStateMachineMode.LEVEL1;
    private Drive drive = Robot.drive;
    private Elevator elevator = Robot.elevator;
    private CargoIntake cargoIntake = Robot.cargoIntake;
    private HatchMechanism hatch = Robot.hatch;
    private HumanInput HI = Robot.HI;
    private TrackingStateMachine trackingStateMachine = Robot.trackingStateMachine;
    private DriverStation ds = DriverStation.getInstance();

    private Timer timer = new Timer();

    private boolean request, force;
    private boolean lastRequest, lastElevatorInPosition, lastForce;

    private int desiredElevatorHeight = 0;

    
    public void update() {
        if((ds.isAutonomous() || HI.getAuto()) && !request) {
            currentState = GamePieceState.WAITING;
        }
        drive.setPlacingOnRocket(placingCargoOnRocket());
        switch(currentState) {
            case WAITING:
                switch(mode) { 
                    case LEVEL1:
                        if(cargoIntake.hasBall()) {
                            desiredElevatorHeight = Constants.kElevatorBallLevel1;
                            nextState = GamePieceState.PLACING_BALL;
                        }
                        else {
                            desiredElevatorHeight = Constants.kElevatorHatchLevel1;
                            nextState = GamePieceState.PLACING_HATCH;
                        }
                        break;
                    case LEVEL2:
                        if(cargoIntake.hasBall()) {
                            desiredElevatorHeight = Constants.kElevatorBallLevel2;
                            nextState = GamePieceState.PLACING_BALL;
                        }
                        else {
                            desiredElevatorHeight = Constants.kElevatorHatchLevel2;
                            nextState = GamePieceState.PLACING_HATCH;
                        }
                        break;
                    case LEVEL3:
                        if(cargoIntake.hasBall()) {
                            desiredElevatorHeight = Constants.kElevatorBallLevel3;
                            nextState = GamePieceState.PLACING_BALL;
                        }
                        else {
                            desiredElevatorHeight = Constants.kElevatorHatchLevel3;
                            nextState = GamePieceState.PLACING_HATCH;
                        }
                        break;
                    case HATCH_PICKUP:
                            desiredElevatorHeight = Constants.kElevatorHatchPickup;
                            nextState = GamePieceState.GRABBING_HATCH;
                        break; 
                    case CARGO_PICKUP:
                            desiredElevatorHeight = Constants.kElevatorCargoStationPickup;
                            nextState = GamePieceState.GRABBING_BALL;
                        break;
                    case CARGO_SHIP:
                        if(cargoIntake.hasBall()) {
                            desiredElevatorHeight = Constants.kElevatorBallCargoShip;
                            nextState = GamePieceState.PLACING_BALL;
                        }
                        else {
                            desiredElevatorHeight = Constants.kElevatorHatchLevel1;
                            nextState = GamePieceState.PLACING_HATCH;
                        }
                        break;
                    }
                if(nextState == GamePieceState.GRABBING_HATCH && trackingStateMachine.isDriving()) {
                    if(drive.getDistanceToTarget() <= 48) {
                        elevator.set(desiredElevatorHeight);
                        hatch.setGripperDown(true);
                    }
                    if(drive.getAtStation() && elevator.inPosition()) {
                        Robot.driverDisabled = true;
                        currentState = nextState;
                        drive.set(0,0);
                    }
                }
                if(request) {
                    if((nextState == GamePieceState.GRABBING_HATCH)) {
                        hatch.setGripperDown(true);
                    }
                    if((nextState == GamePieceState.GRABBING_HATCH || nextState == GamePieceState.GRABBING_BALL) || mode == GamePieceStateMachineMode.CARGO_SHIP) {
                        if(nextState != GamePieceState.PLACING_HATCH)
                            elevator.set(desiredElevatorHeight); 
                        if(drive.getAtStation()) {
                            elevator.set(desiredElevatorHeight);
                            timer.stop();
                            timer.reset();
                            currentState = nextState;
                            Robot.driverDisabled = true;
                            drive.set(0,0);
                        }
                    }
                    else if(drive.getAtRocket() && !(nextState == GamePieceState.GRABBING_HATCH || nextState == GamePieceState.GRABBING_BALL)) {
                        currentState = nextState;
                        Robot.driverDisabled = true;
                        elevator.set(desiredElevatorHeight);
                        timer.stop();
                        timer.reset();
                        drive.set(0,0);
                    }
                }
                if(getForce()) {
                    elevator.set(desiredElevatorHeight);
                    currentState = nextState;
                    Robot.driverDisabled = true;
                    drive.set(0,0);
                }
                break;
            case GRABBING_HATCH:
                drive.set(0,0);
                if(elevator.inPosition())
                    hatch.setIntakeRequest(true);
                if(hatch.isDone() || getForce()) {
                    hatch.setIntakeRequest(false);
                    currentState = GamePieceState.BACKUP_HATCH;
                    drive.resetDriveEncoders();
                }
                break;
            case GRABBING_BALL:
                drive.set(0,0);
                if(elevator.inPosition())
                    cargoIntake.setPickupFromStationRequest(true);
                if(cargoIntake.isDone()|| getForce()) {
                    cargoIntake.setPickupFromStationRequest(false);
                }
                break;
            case PLACING_BALL:
                drive.set(0,0);
                if(elevator.inPosition()) {
                    if(!lastElevatorInPosition)
                        timer.start();
                    cargoIntake.setPlaceRequest(true);
                }
                if(timer.get() >= .25 || getForce()) {
                    cargoIntake.setPlaceRequest(false);
                    currentState = GamePieceState.RETRACT;

                }
                break;
            case PLACING_HATCH:
                drive.set(0,0);
                if(elevator.inPosition())
                    hatch.setPlaceRequest(true);
                if(hatch.isDone() || getForce()) {
                    hatch.setPlaceRequest(false);
                    currentState = GamePieceState.BACKUP_HATCH;
                    drive.resetDriveEncoders();
                }
                break;
            case BACKUP_HATCH:
                drive.setDriveMode(DriveMode.GYROLOCK);
                drive.set(-.25, -.25);
                if(drive.getAverageRioPosition() <= -12 || getForce()) {
                    hatch.setRetractRequest(true);
                    drive.set(0, 0);
                    currentState = GamePieceState.RETRACT;
                }
                break;
            case BACKUP_BALL:
                elevator.set(0);
                cargoIntake.setPlaceRequest(false);
                drive.setDriveMode(DriveMode.GYROLOCK);
                drive.set(-.25, -.25);
                if(drive.getAverageRioPosition() <= -12 || getForce()) {
                    hatch.setRetractRequest(true);
                    drive.set(0, 0);
                    currentState = GamePieceState.DONE;
                }
                break;
            case RETRACT:
                elevator.set(0);
                currentState = GamePieceState.DONE;
                break;
            case DONE:
                if(!request) {
                    currentState = GamePieceState.WAITING;
                }
                break;
        }
        lastRequest = request;
        lastForce = force;
        lastElevatorInPosition = elevator.inPosition();
    }

    
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Game Piece State Machine State", currentState.toString());
        SmartDashboard.putString("Game Piece State Machine Mode", mode.toString());

    }

    public void debug() {
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

    public boolean isPlacing() {
        return currentState == GamePieceState.PLACING_BALL || currentState == GamePieceState.PLACING_HATCH
         || currentState == GamePieceState.GRABBING_HATCH || currentState == GamePieceState.GRABBING_BALL
         || currentState == GamePieceState.BACKUP_BALL || currentState == GamePieceState.BACKUP_HATCH ||
         currentState == GamePieceState.RETRACT;
    }
    
    public boolean placingCargoOnRocket() {
        return desiredElevatorHeight == Constants.kElevatorBallLevel1 ||
        desiredElevatorHeight == Constants.kElevatorBallLevel2 ||
        desiredElevatorHeight == Constants.kElevatorBallLevel3;
    }

    public void reset() {
        currentState = GamePieceState.WAITING;
    }

    public void setForce(boolean force) {
        this.force = force;
    }

    private boolean getForce() {
        return force && !lastForce;
    }
}
