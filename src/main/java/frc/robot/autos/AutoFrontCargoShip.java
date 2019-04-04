package frc.robot.autos;

import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;

public class AutoFrontCargoShip extends Autonomous {

    public enum State {
        START,
        DRIVE,
        STOP,
        PLACE_HATCH,
        DRIVE_BACK,
        TURN_AT_STATION,
        PICKUP_HATCH,
        BACKUP_FROM_STATION,
        DONE
    }

    private State currentState = State.START;

    @Override
    public void reset() {
        currentState = State.START;
    }

    @Override
    public void update() {
        switch(currentState) {
            case START:
                currentState = State.DRIVE;
                drive.resetDriveEncoders();
                break;
            case DRIVE:
                driveGyrolock(.5, getAngle());
                if(drive.getAverageRioPosition() >= 24) {
                    drivePower(0);
                    startTimer();
                    currentState = State.STOP;
                }
                break;
            case STOP:
                if(getTime() >= .25) {
                    currentState = State.PLACE_HATCH;
                }
                break;
            case PLACE_HATCH:
                gamePieceInteract(GamePieceStateMachineMode.CARGO_SHIP, .5);
                if(gamePieceStateMachine.isDone()) {
                    stopGamePieceInteract();
                    drivePower(0);
                    currentState = State.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

}