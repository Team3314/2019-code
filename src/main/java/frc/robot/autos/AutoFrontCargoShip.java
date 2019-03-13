package frc.robot.autos;

import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;

public class AutoFrontCargoShip extends Autonomous {

    public enum State {
        START,
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
                currentState = State.PLACE_HATCH;
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