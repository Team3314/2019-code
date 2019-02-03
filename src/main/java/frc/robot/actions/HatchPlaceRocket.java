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

    public HatchPlaceRocket(int level, boolean far) {
        placeLevel = level;
        placeFar = far;
    }

    int placeLevel;
    boolean placeFar;

    private State currentState;

    @Override
    public void reset() {

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

}
