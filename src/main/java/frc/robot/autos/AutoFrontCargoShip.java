package frc.robot.autos;

import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;

public class AutoFrontCargoShip extends Autonomous {

    public enum State {
        START,
        DRIVE,
        STOP,
        PLACE_HATCH,
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
                gamePieceStateMachine.setRequest(true);
                gamePieceStateMachine.setDriveSpeed(.6);
                gamePieceStateMachine.setMode(GamePieceStateMachineMode.CARGO_SHIP);
                if(gamePieceStateMachine.isDone()) {
                    gamePieceStateMachine.setRequest(false);
                    drivePower(0);
                    currentState = State.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

}