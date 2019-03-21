package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive.DriveMode;

public class TrackingStateMachine {
    public enum State {
        WAITING,
        ALIGNING,
        DRIVING
    }

    private Elevator elevator = Robot.elevator;
    private Drive drive = Robot.drive;

    private double driveSpeed;

    private State currentState = State.WAITING;

    private boolean request;
    private boolean lastRequest;
    public void update() {
        if(!request ||Robot.gamePieceStateMachine.isPlacing()) {
            currentState = State.WAITING;
        }
        switch(currentState) {
            case WAITING:
                if(request && !lastRequest) {
                    drive.setDriveMode(DriveMode.VISION_CONTROL);
                    currentState = State.ALIGNING;
                }
                else {
                    driveSpeed = 0;
                }
                break;
            case ALIGNING:
                if(drive.gyroInPosition()) {
                    drive.setVisionOffset(0);
                    currentState = State.DRIVING;
                }
                break;
            case DRIVING:
                drive.set(driveSpeed, driveSpeed);
                break;
        }
        lastRequest = request;
    }
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Tracking State Machine State", currentState.toString());
    }

    public boolean isDriving() {
        return currentState == State.DRIVING || currentState == State.ALIGNING;
    }

    public void setRequest(boolean request) {
        this.request = request;
    }
    public void setDriveSpeed(double speed) {
        driveSpeed = speed;
    }
	public void debug() {
    }
    public void reset() {
        currentState = State.WAITING;
    }
}