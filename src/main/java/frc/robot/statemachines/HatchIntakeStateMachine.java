package frc.robot.statemachines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;

public class HatchIntakeStateMachine extends StateMachine {

    public enum State {
        WAITING,
        DRIVE,
        LOWER,
        RAISE,
        GRAB,
        DONE
    }

    private Elevator elevator;
    private HatchMechanism hatch;
    private Timer timer = new Timer();

    private State currentState = State.WAITING;

    public HatchIntakeStateMachine() {
        elevator = Robot.elevator;
        hatch = Robot.hatch;
        
    }
    @Override
    public void update() {
        if(!request) {
            currentState = State.WAITING;
        }
        switch(currentState) {
            case WAITING:
                if(request && !lastRequest) {
                    hatch.setGripperDown(true);
                    hatch.setSliderOut(false);
                    elevator.set(Constants.kElevatorHatchPickup);
                    currentState = State.RAISE;
                }
                break;
            case RAISE:
                elevator.set(Constants.kElevatorRaisedHatchPickup);
                if(elevator.inPosition()) {
                    currentState = State.GRAB;
                    timer.start();
                }
                break;
            case GRAB:
                hatch.setGripperDown(false);
                if(timer.get() > .125) {    
                    currentState = State.DONE;
                }
                break;
        }
        lastRequest = request;
    }
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Hatch Intake State Machine State", currentState.toString());
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }
}