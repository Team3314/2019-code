package frc.robot.actions;

public class CargoPlaceRocket extends Action{

    public enum State {
        TURN_TO_DRIVE,
        DRIVE,
        TURN_TO_ROCKET,
        DRIVE_TO_PLACE,
        RELEASE,
        BACKUP
    }

    private State currentState;

    public CargoPlaceRocket(int level) {
        switch(currentState) {
            case TURN_TO_DRIVE:
                break;
            case DRIVE:
                break;
            case TURN_TO_ROCKET:
                break;
            case DRIVE_TO_PLACE:
                break;
            case RELEASE:
                break;
            case BACKUP:
                break;
        }
    }

    @Override
    public void update() {

    }

    @Override
    public Enum<State> getState() {
        return currentState;
    }

    @Override
    public void init() {

    }

}