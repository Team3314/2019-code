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
    public void init() { /* XXX TEMPORARY
        if(Math.abs(getAngle() - 180) <= 20 && isTargetInView()) {
            currentState = State.ALIGN;
        }*/
        setGripperDown(true);
        setSliderOut(false);
        driveGyrolock(.5, getAngle());
        currentState = State.DRIVE;
    }

    @Override
    public void update() {
        switch(currentState) {
            case TURN:
                driveGyrolock(0, 180);
                if(gyroTurnDone()) {
                    turnToTarget();
                    currentState = State.ALIGN;
                }
                break;
            case ALIGN:
                if(isAligned()) {
                    driveGyrolock(.5, getDesiredAngle());
                    currentState = State.DRIVE;
                }
                break;
            case DRIVE:
                if(collision()) {
                    currentState = State.PICKUP;
                }
                break;
            case PICKUP:
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
