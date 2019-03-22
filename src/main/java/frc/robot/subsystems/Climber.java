package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class Climber implements Subsystem {

    public enum State {
        WAITING,
        INTAKE_DOWN,
        INTAKE_AND_CLIMBER_DOWN,
        INTAKE_FURTHER_DOWN,
        DRIVE,
        RAISE_CLIMBER,
        KEEP_DRIVING,
        STOP

    }

    private AHRS navx;
    private Timer timer = new Timer();

    private DoubleSolenoid climberPiston, intakeClimbPiston;
    private Solenoid highPressure;
    private boolean stopClimber = false;

    private boolean climbRequest, previousStateRequest, intakeFurtherDownRequest, autoClimbButton, manualControlClimb = false;
    private boolean lastClimbRequest, lastPreviousStateRequest, lastIntakeFurtherDownRequest = false;

    private Drive drive = Robot.drive;
    private CargoIntake cargoIntake = Robot.cargoIntake;

    private State currentState = State.WAITING;

    public Climber(DoubleSolenoid climberPiston, DoubleSolenoid intakeClimbPiston, Solenoid highPressure, AHRS navx) {
        this.climberPiston = climberPiston;
        this.intakeClimbPiston = intakeClimbPiston;
        this.highPressure = highPressure;
        this.navx = navx;
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
        if(!autoClimbButton && (climbRequest && !lastClimbRequest) || (previousStateRequest && !lastPreviousStateRequest)){
            manualControlClimb = true;
        }
        drive.set(0, 0);
        switch(currentState) {
            case WAITING:
                highPressure.set(false);
                climberPiston.set(Constants.kClimberUp);
                if(!intakeFurtherDownRequest)
                    intakeClimbPiston.set(Constants.kIntakeClimberUp);
                else 
                    intakeClimbPiston.set(Constants.kIntakeClimberDown);
                if(autoClimbButton){
                    navx.reset();
                    manualControlClimb = false;
                    currentState = State.INTAKE_DOWN;
                }
                if(manualControlClimb && climbRequest && !lastClimbRequest){
                    navx.reset();
                    currentState = State.INTAKE_DOWN;
                }
                break;
            case INTAKE_DOWN:
                    highPressure.set(true);
                    climberPiston.set(Constants.kClimberUp);
                    cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                    intakeClimbPiston.set(Constants.kIntakeClimberUp);
                    if(previousStateRequest && !lastPreviousStateRequest) {
                        highPressure.set(false);
                        climberPiston.set(Constants.kClimberUp);
                        cargoIntake.setIntakeState(IntakeState.WAITING);
                        intakeClimbPiston.set(Constants.kIntakeClimberUp);
                        currentState = State.WAITING;
                    }
                    if(manualControlClimb && climbRequest && !lastClimbRequest)
                        currentState = State.INTAKE_AND_CLIMBER_DOWN;
                    else if(!manualControlClimb && navx.getRoll() <= -10)
                        currentState = State.INTAKE_AND_CLIMBER_DOWN;
                break;
            case INTAKE_AND_CLIMBER_DOWN:    
                    highPressure.set(true);
                    climberPiston.set(Constants.kClimberDown);
                    cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                    intakeClimbPiston.set(Constants.kIntakeClimberUp);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.INTAKE_DOWN;
                }
                if(manualControlClimb && climbRequest && !lastClimbRequest)
                    currentState = State.INTAKE_FURTHER_DOWN;
                else if(!manualControlClimb && navx.getRoll() >= -5)
                    currentState = State.INTAKE_FURTHER_DOWN;
                break;
            case INTAKE_FURTHER_DOWN:
                    highPressure.set(true);
                    climberPiston.set(Constants.kClimberDown);
                    cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                    intakeClimbPiston.set(Constants.kIntakeClimberDown);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.INTAKE_AND_CLIMBER_DOWN;
                }
                if(manualControlClimb && climbRequest && !lastClimbRequest)
                    currentState = State.DRIVE;
                /*else if(!manualControlClimb && navx.getRoll() < 4)
                    currentState = State.DRIVE;*/
                break;
            case DRIVE:
                cargoIntake.setIntakeState(IntakeState.DRIVE_BACK);
                drive.set(-.1, -.1);
                if(previousStateRequest && !lastPreviousStateRequest){
                    currentState = State.INTAKE_FURTHER_DOWN;
                }
                if(manualControlClimb && climbRequest && !lastClimbRequest)
                    currentState = State.RAISE_CLIMBER;
                /*else if(!manualControlClimb && !navx.isMoving())
                    currentState = State.RAISE_CLIMBER;*/
                break;
            case RAISE_CLIMBER:
                climberPiston.set(Constants.kClimberUp);
                cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                timer.start();
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.DRIVE;
                }
                if(manualControlClimb && climbRequest && !lastClimbRequest)
                    currentState = State.KEEP_DRIVING;
                /*else if(timer.get() >= 3)
                    currentState = State.KEEP_DRIVING;*/
                break;
            case KEEP_DRIVING:
                cargoIntake.setIntakeState(IntakeState.DRIVE_BACK);
                drive.set(-.05, -.05);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.RAISE_CLIMBER;
                }
                if(manualControlClimb && climbRequest && !lastClimbRequest)
                    currentState = State.STOP;
                /*else if(!manualControlClimb && !navx.isMoving())
                        currentState = State.STOP;*/
                break;
            case STOP:
                cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                if(previousStateRequest && !lastPreviousStateRequest) {
                    currentState = State.KEEP_DRIVING;
                }
                if(climbRequest && !lastClimbRequest)
                    currentState = State.WAITING;
                break;
        }
        timer.stop();
        timer.reset();
        lastClimbRequest = climbRequest;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Climber State", currentState.toString());

    }

    public void setClimbRequest(boolean request) {
        climbRequest = request;
    }

    public void setPreviousStateRequest(boolean request) { 
        previousStateRequest = request;
    }

    public void setAutoClimbButton(boolean autoButton){
        autoClimbButton = autoButton;
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
        SmartDashboard.putBoolean("Climb Request", climbRequest);
        SmartDashboard.putBoolean("Last Climb Request", lastClimbRequest);
        SmartDashboard.putNumber("Roll Angle", navx.getRoll());

    }

    public void setIntakeFurtherDownRequest(boolean request) {
        intakeFurtherDownRequest = request;
    }
}