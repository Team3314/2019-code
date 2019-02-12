package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;

public class HatchIntakeStateMachine {

    public enum State {
        WAITING,
        LOWER,
        EXTEND,
        RAISE,
        GRAB,
        PULL
    }

    private Elevator elevator;
    private HatchMechanism hatch;

    private State currentState = State.WAITING;

    private boolean intakeRequest, lastIntakeRequest;

    public HatchIntakeStateMachine() {
        elevator = Robot.elevator;
        hatch = Robot.hatch;
    }

    public void update() {
        if(!intakeRequest) {
            currentState = State.WAITING;
        }
        switch(currentState) {
            case WAITING:
                hatch.setGripperDown(false);
                hatch.setSliderOut(false);
                if(intakeRequest && !lastIntakeRequest) {
                    currentState = State.LOWER;
                }
                break;
            case LOWER:
                elevator.set(Constants.kElevatorLevel1);
                if(elevator.inPosition()) {
                    currentState = State.EXTEND;
                }
                break;
            case EXTEND:
                hatch.setGripperDown(true);
                hatch.setSliderOut(true);
                currentState = State.RAISE;
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
                    currentState = State.PULL;
                }
                break;
            case PULL:
                hatch.setSliderOut(false);
                currentState = State.WAITING;
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