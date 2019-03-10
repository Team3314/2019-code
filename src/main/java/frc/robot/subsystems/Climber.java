package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class Climber implements Subsystem {

    public enum State {
        WAITING,
        INTAKE_AND_CLIMBER_DOWN,
        INTAKE_FURTHER_DOWN,
        DRIVE,
        RAISE_CLIMBER,
        KEEP_DRIVING,
        STOP

    }

    private DoubleSolenoid climberPiston, intakeClimbPiston;
    private Solenoid highPressure;
    private boolean stopClimber = false;

    private boolean climbRequest, previousStateRequest;
    private boolean lastClimbRequest, lastPreviousStateRequest;

    private Drive drive = Robot.drive;
    private CargoIntake cargoIntake = Robot.cargoIntake;

    private State currentState = State.WAITING;

    public Climber(DoubleSolenoid climberPiston, DoubleSolenoid intakeClimbPiston, Solenoid highPressure) {
        this.climberPiston = climberPiston;
        this.intakeClimbPiston = intakeClimbPiston;
        this.highPressure = highPressure;
    }

    @Override
    public void update() {
        if(stopClimber) {
            highPressure.set(false);
            climberPiston.set(Constants.kClimberUp);
            cargoIntake.setIntakeState(IntakeState.WAITING);
            intakeClimbPiston.set(Constants.kIntakeClimberUp);
            currentState = State.WAITING;
        }
        
        drive.set(0, 0);
        switch(currentState) {
            case WAITING:
                climberPiston.set(Constants.kClimberUp);
                intakeClimbPiston.set(Constants.kIntakeClimberUp);
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.INTAKE_AND_CLIMBER_DOWN;
                }
                break;
            case INTAKE_AND_CLIMBER_DOWN:    
                    highPressure.set(true);
                    climberPiston.set(Constants.kClimberDown);
                    cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                    intakeClimbPiston.set(Constants.kIntakeClimberUp);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    highPressure.set(false);
                    climberPiston.set(Constants.kClimberUp);
                    cargoIntake.setIntakeState(IntakeState.WAITING);
                    intakeClimbPiston.set(Constants.kIntakeClimberUp);
                    currentState = State.WAITING;
                }
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.INTAKE_FURTHER_DOWN;
                }
                break;
            case INTAKE_FURTHER_DOWN:
                    highPressure.set(true);
                    climberPiston.set(Constants.kClimberDown);
                    cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                    intakeClimbPiston.set(Constants.kIntakeClimberDown);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.INTAKE_AND_CLIMBER_DOWN;
                }
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.DRIVE;
                }
                break;
            case DRIVE:
                cargoIntake.setIntakeState(IntakeState.DRIVE_BACK);
                drive.set(-.1, -.1);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.INTAKE_FURTHER_DOWN;
                }
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.RAISE_CLIMBER;
                }
                break;
            case RAISE_CLIMBER:
                climberPiston.set(Constants.kClimberUp);
                cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.DRIVE;
                }
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.KEEP_DRIVING;
                }
                break;
            case KEEP_DRIVING:
                cargoIntake.setIntakeState(IntakeState.DRIVE_BACK);
                drive.set(-.1, -.1);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.RAISE_CLIMBER;
                }
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.STOP;
                }
                break;
            case STOP:
                cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.KEEP_DRIVING;
                }
                if(climbRequest && !lastClimbRequest) {
                    currentState = State.WAITING;
                }
                break;
        }

        lastClimbRequest = climbRequest;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Climber State", currentState.toString());
        SmartDashboard.putBoolean("Climb Request", climbRequest);
        SmartDashboard.putBoolean("Last Climb Request", lastClimbRequest);

    }

    public void setClimbRequest(boolean request) {
        climbRequest = request;
    }

    public void setPreviousStateRequest(boolean request) { 
        previousStateRequest = request;
    }

    public void setStopClimb(boolean stop) {
        stopClimber = stop;
    }

    public boolean isClimbing() {
        return currentState != State.WAITING;
    }

    @Override
    public void resetSensors() {

    }

    @Override
    public void debug() {

    }

    

    

}