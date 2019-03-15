package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.statemachines.GamePieceStateMachine.GamePieceState;
import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;
import frc.robot.subsystems.Drive.DriveMode;

public class AutoTwoHatchRocketClose extends Autonomous {

    public enum State {
        START,
        TURN_TO_ROCKET1,
        STOP,
        PLACE_HATCH1, // Places hatch on first level of rocket
        TURN_TO_STATION1, // Turns to 180 degrees to face loading station
        STOP2,
        DRIVE,
        PICKUP_HATCH, // Grabs hatch from loading station
        BACKUP,
        TURN_TO_ROCKET2, // Turns to 0 degrees to face rocket
        STOP3,
        PLACE_HATCH2, // Places hatch on second level of rocket
        TURN_TO_STATION2, // Turns to 180 degrees to face loading station
        DRIVE_AT_STATION2 // Drives until it is 60 inches from loading station

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
                if(getStartPos() == "StartR")
                    driveGyrolock(0, -30, DriveMode.GYROLOCK_LEFT);
                else if(getStartPos() == "StartL") 
                    driveGyrolock(0, 30, DriveMode.GYROLOCK_RIGHT);
                setHighGear(false);
                currentState = State.STOP;
                break;
            case STOP:
                if(gyroTurnDone()) {
                    startTimer();
                    currentState = State.TURN_TO_ROCKET1;
                }
                break;
            case TURN_TO_ROCKET1:
                if(getTime() >=.5) {
                    resetTimer();
                    currentState = State.PLACE_HATCH1;
                }
                break;
            case PLACE_HATCH1:
                setGamePieceRequest(true);
                gamePieceStateMachine.setDriveSpeed(.75);
                gamePieceStateMachine.setMode(GamePieceStateMachineMode.LEVEL1);
                if(gamePieceStateMachine.getCurrentState() == GamePieceState.BACKUP_HATCH) {
                    hatch.setSliderOut(false);
                    if(getStartPos() == "StartR")
                        driveGyrolock(0, -180, DriveMode.GYROLOCK_RIGHT);
                    else if(getStartPos() == "StartL") 
                        driveGyrolock(0, 180, DriveMode.GYROLOCK_LEFT);
                    setGamePieceRequest(false);
                    currentState = State.TURN_TO_STATION1;
                }
                break;
            case TURN_TO_STATION1:
                if(gyroTurnDone()) {
                    startTimer();
                    currentState = State.STOP2;
                }
                break;
            case STOP2:
                if(getTime() >= .05) {
                    resetTimer();
                    //driveGyrolock(.75, 180, 60);
                    currentState = State.PICKUP_HATCH;
                }
                break;
            case DRIVE:
                if(gyroTurnDone()) {
                    drive.set(0,0);
                    setGamePieceRequest(true);
                    gamePieceStateMachine.setMode(GamePieceStateMachineMode.PICKUP);
                    currentState = State.PICKUP_HATCH;
                }
                break;
            case PICKUP_HATCH:
                gamePieceStateMachine.setDriveSpeed(.6);
                if(gamePieceStateMachine.getCurrentState() == GamePieceState.BACKUP_HATCH) {
                    setGamePieceRequest(false);
                    currentState = State.BACKUP;
                    drive.resetDriveEncoders();
                    driveGyrolock(-1, 180, -54);
                }
                break;
            case BACKUP:
                if(drive.getGyroDriveDone()) {
                    currentState = State.TURN_TO_ROCKET2;
                    if(getStartPos() == "StartR")
                        driveGyrolock(0, -20, DriveMode.GYROLOCK);
                    else if(getStartPos() == "StartL") 
                        driveGyrolock(0, 20, DriveMode.GYROLOCK);
                }
                break;
            case TURN_TO_ROCKET2: 
                if(gyroTurnDone()) {
                    startTimer();
                    drivePower(0);
                    currentState = State.STOP3;
                }
                break;
            case STOP3:
                if(getTime() > .5) {
                    setGamePieceRequest(true);
                    gamePieceStateMachine.setMode(GamePieceStateMachineMode.LEVEL2);
                    resetTimer();
                    currentState = State.PLACE_HATCH2;
                }
                break;
            case PLACE_HATCH2:
                gamePieceStateMachine.setDriveSpeed(.6);
                if(gamePieceStateMachine.isDone()) {
                    setGamePieceRequest(false);
                    currentState = State.TURN_TO_STATION2;
                    driveGyrolock(0, 180, DriveMode.GYROLOCK);
                }
                break;
            case TURN_TO_STATION2:
                if(gyroTurnDone()) {
                    setGamePieceRequest(true);
                    gamePieceStateMachine.setMode(GamePieceStateMachineMode.PICKUP);
                    currentState = State.DRIVE_AT_STATION2;
                }
                break;
            case DRIVE_AT_STATION2:
                if(getCameraDistance() < 72) {
                    setGamePieceRequest(false);
                }
                break;
        }
        SmartDashboard.putString("Auto State", currentState.toString());
    }

}