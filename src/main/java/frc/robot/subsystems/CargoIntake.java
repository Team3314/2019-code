package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
        INTAKE_DOWN,
        CLIMB,
        OVERRIDE
    }

    private Transmission intake, outtake;
    private DoubleSolenoid pivot, climb;
    private Solenoid highPressure;

    private double intakeSpeed, outtakeSpeed;

    private Value pivotState = Constants.kIntakeUp, climbState = Constants.kIntakeClimberUp;

    private boolean high = false;

    private AnalogInput intakeCargoSensor = new AnalogInput(0), elevatorCargoSensor = new AnalogInput(1);
    private DigitalInput raisedSensor = new DigitalInput(4);
    private IntakeState currentIntakeState = IntakeState.WAITING;
    SpeedControllerMode intakeControlMode = SpeedControllerMode.kDutyCycle;
    SpeedControllerMode outtakeControlMode = SpeedControllerMode.kDutyCycle;

    public CargoIntake(Transmission intake, Transmission outtake, DoubleSolenoid pivotPiston, DoubleSolenoid climb, Solenoid highPressure) {
        this.intake = intake;
        this.outtake = outtake;
        pivot = pivotPiston;
        this.climb = climb;
        this.highPressure = highPressure;
    }

    @Override
    public void update() {
        high = false;
        setIntakeSpeed(0);
        setOuttakeSpeed(0);
        switch (currentIntakeState) {
            case WAITING:
                setIntakeDown(false);
                break;
            case INTAKING:
                setIntakeDown(true);
                setIntakeSpeed(1);
                break;
            case RAISING:
                setIntakeDown(false);
                break;
            case TRANSFERRING:
                setIntakeSpeed(1);
                setOuttakeSpeed(.25);
                setIntakeDown(false);
                break;
            case PLACE:
                setOuttakeSpeed(1);
                break;
            case VOMIT:
                setIntakeSpeed(-1);
                break;
            case INTAKE_DOWN:
                high = false;
                setClimbDown(false);
                setIntakeDown(true);
                break;
            case CLIMB:
                high = true;
                setClimbDown(true);
                setIntakeDown(true);
                break;
            case OVERRIDE:
                setIntakeDown(true);
                setIntakeSpeed(1);
                break;
        }
        highPressure.set(high);
        climb.set(climbState);
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
    private void setClimbDown(boolean down) {
        if(down) 
            climbState = Constants.kClimberDown;
        else
            climbState = Constants.kClimberUp;
    }

    /**
     * @param currentState the currentState to set
     */
    public void setIntakeState(IntakeState mode) {
        currentIntakeState = mode;
    }

    /**
     * @return the currentState
     */
    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Cargo Intake current", intake.getOutputCurrent(0));
        SmartDashboard.putNumber("Cargo Intake speed", intakeSpeed);
        SmartDashboard.putString("Cargo Intake state", getIntakeState().toString());
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