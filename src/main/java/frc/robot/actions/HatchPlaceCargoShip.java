package frc.robot.actions;

public class HatchPlaceCargoShip extends Action {

    public enum State {
        TURN_TO_DRIVE,
        DRIVE,
        TURN_TO_CARGO,
        DRIVE_TO_PLACE,
        RELEASE

    }

    private State currentState;

    @Override
    public void update() {
        currentState = State.TURN_TO_DRIVE;
    }

    @Override
    public Enum<State> getState() {
        return currentState;
    }

    @Override
    public void init() {
        
    }
}