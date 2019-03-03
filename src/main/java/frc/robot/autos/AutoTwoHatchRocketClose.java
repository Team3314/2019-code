package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;

public class AutoTwoHatchRocketClose extends Autonomous {

    public enum State {
        START,
        TURN_TO_ROCKET1,
        PLACE_HATCH1, // Places hatch on first level of rocket
        TURN_TO_STATION1, // Turns to 180 degrees to face loading station
        PICKUP_HATCH, // Grabs hatch from loading station
        TURN_TO_ROCKET2, // Turns to 0 degrees to face rocket
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
                driveGyrolock(0, -30);
                setHighGear(false);
                currentState = State.TURN_TO_ROCKET1;
                break;
            case TURN_TO_ROCKET1:
                if(gyroTurnDone()) {
                    currentState = State.PLACE_HATCH1;
                }
                break;
            case PLACE_HATCH1:
                setGamePieceRequest(true);
                gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_LEVEL1);
                if(gamePieceStateMachine.isDone()) {
                    driveGyrolock(0, 180);
                    setGamePieceRequest(false);
                    currentState = State.TURN_TO_STATION1;
                }
                break;
            case TURN_TO_STATION1:
                if(gyroTurnDone()) {
                    setGamePieceRequest(true);
                    gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_PICKUP);
                    currentState = State.PICKUP_HATCH;
                }
                break;
            case PICKUP_HATCH:
                if(gamePieceStateMachine.isDone()) {
                    setGamePieceRequest(false);
                    currentState = State.TURN_TO_ROCKET2;
                    driveGyrolock(0, 0);
                }
                break;
            case TURN_TO_ROCKET2: 
                if(gyroTurnDone()) {
                    setGamePieceRequest(true);
                    gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_LEVEL2);
                    currentState = State.PLACE_HATCH2;
                }
                break;
            case PLACE_HATCH2:
                if(gamePieceStateMachine.isDone()) {
                    setGamePieceRequest(false);
                    currentState = State.TURN_TO_STATION2;
                    driveGyrolock(0, 180);
                }
                break;
            case TURN_TO_STATION2:
                if(gyroTurnDone()) {
                    setGamePieceRequest(true);
                    gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_PICKUP);
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