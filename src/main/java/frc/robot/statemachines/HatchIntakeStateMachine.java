package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Drive.DriveMode;

public class HatchIntakeStateMachine {

    public enum State {
        WAITING,
        DRIVE,
        LOWER,
        RAISE,
        GRAB
    }

    private Elevator elevator;
    private HatchMechanism hatch;
    private Drive drive;
    private HumanInput HI;

    private State currentState = State.WAITING;

    private boolean intakeRequest, lastIntakeRequest;

    public HatchIntakeStateMachine() {
        elevator = Robot.elevator;
        hatch = Robot.hatch;
        drive = Robot.drive;
        HI = Robot.HI;
        
    }

    public void update() {
        if(!intakeRequest) {
            currentState = State.WAITING;
        }
        switch(currentState) {
            case WAITING:
                if(intakeRequest && !lastIntakeRequest) {
                    hatch.setGripperDown(true);
                    hatch.setSliderOut(false);
                    elevator.set(Constants.kElevatorPickup);
                    currentState = State.DRIVE;
                }
                break;
            case RAISE:
                elevator.set(Constants.kElevatorRaisedPickup);
                if(elevator.inPosition()) {
                    currentState = State.GRAB;
                }
                break;
            case GRAB:
                hatch.setGripperDown(false);
                if(hatch.getHasHatch()) {    
                    currentState = State.WAITING;
                }
                break;
        }
        lastIntakeRequest = intakeRequest;
    }

    public void setIntakeRequest(boolean request) {
        intakeRequest = request;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Hatch Intake State Machine State", currentState.toString());
    }
}