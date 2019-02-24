package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Climber implements Subsystem {

    public enum State {
        WAITING,
        INTAKE_DOWN,    
        CLIMBER_DOWN,
        INTAKE_FURTHER_DOWN,
        DRIVE,
        RAISE_CLIMBER
    }

    private DoubleSolenoid climberPiston, intakePiston, intakeClimbPiston;
    private Solenoid highPressure;
    private boolean climberDown;

    private boolean climbRequest, nextStateRequest;
    private boolean lastClimbRequest, lastNextStateRequest;

    private State currentState = State.WAITING;

    public Climber(DoubleSolenoid climberPiston, DoubleSolenoid intakePiston, DoubleSolenoid intakeClimbPiston, Solenoid highPressure) {
        this.climberPiston = climberPiston;
    }

    @Override
    public void update() {
        if(!climbRequest && lastClimbRequest) {
            currentState = State.WAITING;
        }
        switch(currentState) {
            case WAITING:
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.INTAKE_DOWN;
                }
                break;
            case INTAKE_DOWN:
                if(nextStateRequest && !lastNextStateRequest) {
                    currentState = State.CLIMBER_DOWN;
                }
                break;
            case CLIMBER_DOWN:
                if(nextStateRequest && !lastNextStateRequest) {
                    currentState = State.INTAKE_FURTHER_DOWN;
                }
                break;
            case INTAKE_FURTHER_DOWN:
                if(nextStateRequest && !lastNextStateRequest) {
                    currentState = State.DRIVE;
                }
                break;
            case DRIVE:
                if(nextStateRequest && !lastNextStateRequest) {
                    currentState = State.RAISE_CLIMBER;
                }
                break;
            case RAISE_CLIMBER:
                break;
            
        }

        lastNextStateRequest = nextStateRequest;
        lastClimbRequest = climbRequest;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Climber State", currentState.toString());

    }

    public void setClimbRequest(boolean request) {
        climbRequest = request;
    }

    public void setNextStateRequest(boolean request) {
        nextStateRequest = request;
    }

    public void setClimberDown(boolean down) {
        climberDown = down;
    }

    @Override
    public void resetSensors() {

    }

    

    

}