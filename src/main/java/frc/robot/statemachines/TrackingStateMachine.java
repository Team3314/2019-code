package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveMode;

public class TrackingStateMachine extends StateMachine {
    public enum State {
        WAITING,
        ALIGNING,
        DRIVING
    }

    private Camera camera = Robot.camera;
    private Drive drive = Robot.drive;


    private State currentState = State.WAITING;

    @Override
    public void update() {
        if(!request) 
            currentState = State.WAITING;
        switch(currentState) {
            case WAITING:
                if(request && !lastRequest) {
                    drive.setDriveMode(DriveMode.GYROLOCK);
                    currentState = State.ALIGNING;
                }
                break;
            case ALIGNING:
                drive.setDesiredAngle(drive.getAngle() + camera.getTargetHorizError());
                if(drive.gyroInPosition()) {
                    currentState = State.DRIVING;
                }
                break;
            case DRIVING:
                if(camera.isTargetInView()) { 
                    drive.setDesiredAngle(drive.getAngle() + camera.getTargetHorizError());
                }
                else {
                    drive.setDesiredAngle(drive.getDesiredAngle());
                }
                if(drive.collision()) {
                    currentState = State.WAITING;
                }
                break;
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Tracking State Machine State", currentState.toString());
    }


}