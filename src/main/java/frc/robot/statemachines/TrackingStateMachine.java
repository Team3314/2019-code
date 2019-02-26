package frc.robot.statemachines;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveMode;

public class TrackingStateMachine {
    public enum State {
        WAITING,
        ALIGNING,
        DRIVING,
        DONE
    }

    private Drive drive = Robot.drive;
    private HumanInput HI = Robot.HI;

    private AnalogInput distanceSensor;

    private boolean trackingRequest, lastTrackingRequest;

    public TrackingStateMachine(AnalogInput distanceSensor) {
        this.distanceSensor = distanceSensor;
    }
    private State currentState = State.WAITING;

    public void update() {
        if(!trackingRequest && lastTrackingRequest) 
            currentState = State.WAITING;
        switch(currentState) {
            case WAITING:
                if(trackingRequest && !lastTrackingRequest) {
                    drive.setDriveMode(DriveMode.VISION_CONTROL);
                    currentState = State.ALIGNING;
                }
                break;
            case ALIGNING:
                if(drive.gyroInPosition()) {
                    currentState = State.DRIVING;
                }
                break;
            case DRIVING:
                if(drive.collision()) {
                    drive.setDriveMode(DriveMode.OPEN_LOOP);
                    currentState = State.DONE;
                }
                break;
            case DONE:
                
                break;
        }
        lastTrackingRequest = trackingRequest;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Tracking State Machine State", currentState.toString());
        SmartDashboard.putBoolean("Distance Sensor Triggered", distanceSensorTriggered());
    
    }

    public boolean isDone() {
        return currentState == State.DONE;
    }

    public void setTrackingRequest(boolean request) {
        trackingRequest = request;
    }
    public boolean distanceSensorTriggered() {
        return distanceSensor.getVoltage() > Constants.kCargoSensorVoltageThreshold;
    }

}