package frc.robot.actions;

public class HatchPlaceRocket extends Action {

    public enum State {
        DRIVE_BACK,
        TURN_TO_ROCKET,
        ALIGN,
        DRIVE_AT_TARGET,
        PLACE,
        BACKUP,
        DONE
    }

    private int placeLevel;
    private boolean placeFar;

    private State currentState;

    public HatchPlaceRocket(int level, boolean far) {
        currentState = State.TURN_TO_ROCKET;
        placeLevel = level;
        placeFar = far;
    }

    public HatchPlaceRocket(int level) {
        placeLevel = level;
        currentState = State.ALIGN;
    }

    @Override
    public void update() {
        switch(currentState) {
            case DRIVE_BACK:
                startPathFollower(null); //TODO ADD PATHS
                break;
            case TURN_TO_ROCKET:
                if(placeFar) {
                    driveGyrolock(0, 180);
                }
                else 
                    driveGyrolock(0, 0);
                break;
            case ALIGN:
                break;
            case DRIVE_AT_TARGET:
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
