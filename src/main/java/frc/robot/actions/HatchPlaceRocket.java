package frc.robot.actions;

public class HatchPlaceRocket extends Action {

    public enum State {
        TURN,
        ALIGN,
        DRIVE,
        PLACE,
        BACKUP,
        DONE
    }

    private int placeLevel;
    private boolean placeFar;

    private State currentState;

    public HatchPlaceRocket(int level, boolean far) {
        currentState = State.TURN;
        placeLevel = level;
        placeFar = far;
    }

    @Override
    public void update() {
        switch(currentState) {
            case TURN:
                break;
            case ALIGN:
                break;
            case DRIVE:
                break;
            case PLACE:
                break;
            case BACKUP:
                break;
            case DONE:
                break;
        }
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }

    @Override
    public Enum<State> getState() {
        return currentState;
    }

    @Override
    public void init() {

    }

}
