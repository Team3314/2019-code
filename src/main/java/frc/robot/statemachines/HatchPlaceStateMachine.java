package frc.robot.statemachines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.HatchMechanism;

public class HatchPlaceStateMachine extends StateMachine {

    public enum State {
        WAITING,
        EXTEND,
        RETRACT,
        DONE
    }

    private State currentState = State.WAITING;
    private HatchMechanism hatch = Robot.hatch;

    private Timer timer = new Timer();

    @Override
    public void update() {
        if(!request && lastRequest) {
            currentState = State.WAITING;
            timer.stop();
            timer.reset();
        }
        switch(currentState) {
            case WAITING:
                if(request && !lastRequest) {
                    currentState = State.EXTEND;
                    hatch.setGripperDown(false);
                    hatch.setSliderOut(true);
                    timer.start();
                }
                break;
            case EXTEND:
                if(timer.get() > .1) {
                    currentState = State.RETRACT;
                    hatch.setSliderOut(false);
                    timer.reset();
                }
                break;
            case RETRACT:
                if(timer.get() > .1) {
                    currentState = State.DONE;
                    hatch.setGripperDown(false);
                    timer.reset();
                    timer.stop();
                }
                break;
            case DONE:
                break;
        }

        lastRequest = request;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Hatch Place State Machine State", currentState.toString());
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }

}