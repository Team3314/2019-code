package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.infrastructure.SpeedControllerMode;
import frc.robot.infrastructure.Transmission;

public class CargoIntake implements Subsystem {

    public enum IntakeState {
        WAITING,
        INTAKING,
        RAISING,
        TRANSFERRING,
        VOMIT,
        PLACE,
        OVERRIDE
    }

    private Transmission intake, outtake;
    private DoubleSolenoid pivot;

    private double intakeSpeed, outtakeSpeed;

    private Value pivotState = Constants.kIntakeUp;

    private boolean intakeRequest, placeRequest, vomitRequest;
    private boolean lastIntakeRequest, lastPlaceRequest, lastVomitRequest;

    private AnalogInput intakeCargoSensor = new AnalogInput(0), elevatorCargoSensor = new AnalogInput(1);
    private DigitalInput raisedSensor = new DigitalInput(4);
    private IntakeState currentIntakeState = IntakeState.WAITING;
    private Elevator elevator = Robot.elevator;
    SpeedControllerMode intakeControlMode = SpeedControllerMode.kDutyCycle;
    SpeedControllerMode outtakeControlMode = SpeedControllerMode.kDutyCycle;

    public CargoIntake(Transmission intake, Transmission outtake, DoubleSolenoid pivotPiston) {
        this.intake = intake;
        this.outtake = outtake;
        pivot = pivotPiston;
    }

    @Override
    public void update() {
        if((!intakeRequest && lastIntakeRequest) ||
         (!placeRequest && lastPlaceRequest) || 
         (!vomitRequest && lastVomitRequest)) {
            currentIntakeState = IntakeState.WAITING;
        }
        setIntakeSpeed(0);
        setOuttakeSpeed(0);
        switch (currentIntakeState) {
            case WAITING:
                if(vomitRequest && !lastVomitRequest) {
                    currentIntakeState = IntakeState.VOMIT;
                }
                if(placeRequest && !lastPlaceRequest) {
                    currentIntakeState = IntakeState.PLACE;
                }
                if(intakeRequest && !lastIntakeRequest) {
                    currentIntakeState = IntakeState.INTAKING;
                }
                break;
            case INTAKING:
                setIntakeDown(true);
                setIntakeSpeed(1);
                elevator.set(Constants.kElevatorBallLevel1);
                if(getCargoInIntake()) {
                    currentIntakeState = IntakeState.RAISING;
                }
                break;
            case RAISING:
                setIntakeDown(false);
                if(elevator.inPosition() && getIsUp()) {
                    currentIntakeState = IntakeState.TRANSFERRING;
                }
                break;
            case TRANSFERRING:
                setIntakeDown(false);
                setOuttakeSpeed(.25);
                setIntakeSpeed(1);
                if(getCargoInCarriage()) {
                    currentIntakeState = IntakeState.WAITING;
                }
                break;
            case PLACE:
                setOuttakeSpeed(1);
                break;
            case VOMIT:
                setIntakeSpeed(-1);
                break;
            case OVERRIDE:
                setIntakeDown(true);
                setIntakeSpeed(1);
                break;
        }
        
        lastIntakeRequest = intakeRequest;
        lastPlaceRequest = placeRequest;
        lastVomitRequest = vomitRequest;

        pivot.set(pivotState);
        intake.set(intakeSpeed, intakeControlMode);
        outtake.set(outtakeSpeed, outtakeControlMode);
    }

    /**
     * @param desiredSpeed the desiredSpeed to set
     */
    private void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }
    private void setOuttakeSpeed(double speed) {
        this.outtakeSpeed = speed;
    }

    private void setIntakeDown(boolean down) {
        if(down) {
            pivotState = Constants.kIntakeDown;
        }
        else {
            pivotState = Constants.kIntakeUp;
        }
    }
    /**
     * @param currentIntakeState the currentIntakeState to set
     */
    public void setIntakeState(IntakeState mode) {
        currentIntakeState = mode;
    }

    /**
     * @return the currentIntakeState
     */
    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Cargo Intake current", intake.getOutputCurrent(0));
        SmartDashboard.putNumber("Cargo Intake speed", intakeSpeed);
        SmartDashboard.putString("Cargo Intake IntakeState", getIntakeState().toString());
        SmartDashboard.putNumber("Cargo intake sensor voltage", intakeCargoSensor.getVoltage());
        SmartDashboard.putBoolean("Ball in Intake", getCargoInIntake());
        SmartDashboard.putBoolean("Ball in Carriage", getCargoInCarriage());
        SmartDashboard.putBoolean("Cargo Intake Raised", getIsUp());
    }

    @Override
    public void resetSensors() {
        setIntakeState(IntakeState.WAITING);
    }

    public boolean getCargoInIntake() {
        return intakeCargoSensor.getVoltage() < Constants.kCargoSensorVoltageThreshold;
    }

    public boolean getCargoInCarriage() {
        return elevatorCargoSensor.getVoltage() < Constants.kCargoSensorVoltageThreshold;
    }
    public boolean getIsUp() {
        return !raisedSensor.get();
    }
}