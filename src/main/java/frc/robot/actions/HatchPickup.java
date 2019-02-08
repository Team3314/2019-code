package frc.robot.actions;

public class HatchPickup extends Action {

    public enum State {
        TURN,
        ALIGN,
        DRIVE,
        PICKUP,
        BACKUP,
        DONE
    }

    private State currentState;

    public HatchPickup() {
        currentState = State.TURN;
    }

    @Override
    public void init() {
        if(Math.abs(getAngle()) <= 5 && isTargetInView()) {
            currentState = State.ALIGN;
        }
    }

    @Override
    public void update() {
        switch(currentState) {
            case TURN:
                driveGyrolock(0, 180);
                if(gyroTurnDone()) 
                    currentState = State.ALIGN;
                break;
            case ALIGN:
                if(isAligned())
                    currentState = State.DRIVE;
                break;
            case DRIVE:
                if(collision()) {
                    currentState = State.PICKUP;
                }
                break;
            case PICKUP:
                if(hasHatch()) {
                    currentState = State.BACKUP;
                    drivePosition(0);
                }
                break;
            case BACKUP:
                if(driveInPosition()) {
                    currentState = State.DONE;
                }
                break;
            case DONE:
                isDone = true;
                break;
        }
    }
    @Override
    public Enum<State> getState() {
        return currentState;
    }

}
