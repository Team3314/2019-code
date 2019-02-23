package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveMode;

public class TrackingStateMachine extends StateMachine {
    public enum State {
        WAITING,
        ALIGNING,
        DRIVING,
        DONE
    }

    private Camera camera = Robot.camera;
    private Drive drive = Robot.drive;
    private HumanInput HI = Robot.HI;


    private State currentState = State.WAITING;

    @Override
    public void update() {
        if(!request && lastRequest) 
            currentState = State.WAITING;
        switch(currentState) {
            case WAITING:
                if(request && !lastRequest) {
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
                drive.setTank(HI.getLeftThrottle(), HI.getLeftThrottle(), 2);
                if(drive.collision()) {
                    drive.setDriveMode(DriveMode.OPEN_LOOP);
                    currentState = State.DONE;
                }
                break;
            case DONE:
                
                break;
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Tracking State Machine State", currentState.toString());
    
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }


}