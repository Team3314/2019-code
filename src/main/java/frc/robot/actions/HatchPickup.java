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
    public void reset() {

    }

    @Override
    public void update() {
        switch(currentState) {
            case TURN:
                driveGyrolock(0, 30);
                if(gyroTurnDone()) 
                    currentState = State.ALIGN;
                break;
            case ALIGN:
                if()
                    currentState = State.Drive;
                break;
            case DRIVE:
                if()
                    currentState = State.PICKUP;
                break;
            case PICKUP:
                if()
                    currentState = State.BACKUP;
                break;
            case BACKUP:
                if()
                    currentState = State.DONE;
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
