package frc.robot.autos;

public class AutoTwoHatchRocketClose extends Autonomous {

    public enum State {
        START,
        ALIGN_TO_ROCKET1, //Aligns with vision target on rocket
        DRIVE_AT_ROCKET1, // Drives at rocket until collision
        PLACE_HATCH1, // Places hatch on first level of rocket
        BACKUP1, // Backs up 12 inches from rocket
        TURN_TO_STATION1, // Turns to 180 degrees to face loading station
        ALIGN_TO_STATION1, // Aligns with vision target on loading station
        DRIVE_AT_STATION1, // Drives at loading station until collision
        PICKUP_HATCH, // Grabs hatch from loading station
        BACKUP2, // Backs up 12 inches from loading station
        TURN_TO_ROCKET, // Turns to 0 degrees to face rocket
        ALIGN_TO_ROCKET2, // Aligns with vision target on rocket
        DRIVE_AT_ROCKET2, // Drives at rocket until collision
        PLACE_HATCH2, // Places hatch on second level of rocket
        BACKUP3, // Backs up 12 inches from rocket
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
                break;
            case ALIGN_TO_ROCKET1:
                break;
            case DRIVE_AT_ROCKET1:
                break;
            case PLACE_HATCH1:
                break;
            case BACKUP1:
                break;
            case TURN_TO_STATION1:
                break;
            case ALIGN_TO_STATION1:
                break;
            case DRIVE_AT_STATION1:
                break;
            case PICKUP_HATCH:
                break;
            case BACKUP2:
                break;
            case TURN_TO_ROCKET:
                break;
            case ALIGN_TO_ROCKET2:
                break;
            case DRIVE_AT_ROCKET2:
                break;
            case PLACE_HATCH2:
                break;
            case BACKUP3:
                break;
            case TURN_TO_STATION2:
                break;
            case DRIVE_AT_STATION2:
                break;
        }

    }

}