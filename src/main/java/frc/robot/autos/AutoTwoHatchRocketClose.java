package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.statemachines.GamePieceStateMachine.GamePieceState;
import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;
import frc.robot.subsystems.Drive.DriveMode;

public class AutoTwoHatchRocketClose extends Autonomous {

    public enum State {
        START,
        PLACE_HATCH1, // Places hatch on first level of rocket
        TURN_TO_STATION1, // Turns to 180 degrees to face loading station
        PICKUP_HATCH, // Grabs hatch from loading station
        BACKUP,
        TURN_TO_ROCKET2, // Turns to 0 degrees to face rocket
        STOP2,
        PLACE_HATCH2, // Places hatch on second level of rocket
        TURN_TO_STATION2, // Turns to 180 degrees to face loading station
        DRIVE_AT_STATION2, // Drives until it is 60 inches from loading station
        DONE
    }

    private State currentState = State.START;

    @Override
    public void reset() {
        currentState =State.START;
    }


    @Override
    public void update() {
        switch(currentState) {
            case START:
                setHighGear(false);
                resetSensors();
                currentState = State.PLACE_HATCH1;
                break;
            case PLACE_HATCH1:
                gamePieceInteract(GamePieceStateMachineMode.LEVEL1, .65);
                if(gamePieceStateMachine.getCurrentState() == GamePieceState.BACKUP_HATCH) {
                    stopGamePieceInteract();
                    hatch.setSliderOut(false);
                    if(getStartPos() == "StartR")
                        driveGyrolock(0, -145, DriveMode.GYROLOCK_RIGHT);
                    else if(getStartPos() == "StartL") 
                        driveGyrolock(0, 145, DriveMode.GYROLOCK_LEFT);
                    currentState = State.TURN_TO_STATION1;
                }
                break;
            case TURN_TO_STATION1:
                if(gyroTurnDone()) {
                    startTimer();
                    currentState = State.PICKUP_HATCH;
                }
                break;
            case PICKUP_HATCH:
                gamePieceInteract(GamePieceStateMachineMode.HATCH_PICKUP, .65);
                if(gamePieceStateMachine.getCurrentState() == GamePieceState.BACKUP_HATCH) {
                    stopGamePieceInteract();
                    currentState = State.BACKUP;
                    drive.resetDriveEncoders();
                }
                break;
            case BACKUP:
                    if(getStartPos() == "StartR")
                        driveGyrolock(-1, -145);
                    else if(getStartPos() == "StartL") 
                        driveGyrolock(-1, 145);
                if(drive.getAverageRioPosition() <= -54) {
                    currentState = State.TURN_TO_ROCKET2;
                    if(getStartPos() == "StartR")
                        driveGyrolock(0, -10, DriveMode.GYROLOCK);
                    else if(getStartPos() == "StartL") 
                        driveGyrolock(0, 10, DriveMode.GYROLOCK);
                }
                break;
            case TURN_TO_ROCKET2: 
                if(gyroTurnDone()) {
                    elevator.set(Constants.kElevatorHatchLevel2);
                    startTimer();
                    drivePower(0);
                    currentState = State.STOP2;
                }
                break;
            case STOP2:
                if(getTime() > .25) {
                    resetTimer();
                    currentState = State.PLACE_HATCH2;
                }
                break;
            case PLACE_HATCH2:
                gamePieceInteract(GamePieceStateMachineMode.LEVEL2, .65);
                if(gamePieceStateMachine.isDone()) {
                    stopGamePieceInteract();
                    currentState = State.TURN_TO_STATION2;
                    if(getStartPos() == "StartR")
                        driveGyrolock(0, -150);
                    else if(getStartPos() == "StartL") 
                        driveGyrolock(0, 150);
                }
                break;
            case TURN_TO_STATION2:
                if(gyroTurnDone()) {
                    currentState = State.DRIVE_AT_STATION2;
                }
                break;
            case DRIVE_AT_STATION2:
                gamePieceInteract(GamePieceStateMachineMode.HATCH_PICKUP, .6);
                if(getCameraDistance() < 72) {
                    stopGamePieceInteract();
                    currentState = State.DONE;
                }
                break;
            case DONE:
                break;
        }
        SmartDashboard.putString("Auto State", currentState.toString());
    }

}