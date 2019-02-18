package frc.robot.actions;

public class CargoPlaceCargoShip extends Action{

    public enum State { 
        TURN_TO_DRIVE,
        DRIVE,
        TURN_TO_CARGO,
        DRIVE_TO_PLACE,
        RELEASE,
        BACKUP
    }

    private State currentState;

    @Override
    public void update() {
        switch(currentState) {
            case TURN_TO_DRIVE:
                break;
            case DRIVE:
                break;
            case TURN_TO_CARGO:
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
    public Enum<State> getState() {
        return currentState;
    }

    @Override
    public void init() {

    }

}