package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
        OVERRIDE
    }

    private Transmission intake, outtake;
    private DoubleSolenoid pivot;

    private double intakeSpeed, outtakeSpeed;

    private boolean cargoInIntake, cargoInCarriage, lastCargoInIntake;

    private Value pivotState = Constants.kIntakeUp;

    private AnalogInput cargoSensor = new AnalogInput(0);
    private IntakeState currentIntakeState = IntakeState.WAITING;
    SpeedControllerMode intakeControlMode = SpeedControllerMode.kDutyCycle;
    SpeedControllerMode outtakeControlMode = SpeedControllerMode.kDutyCycle;

    public CargoIntake(Transmission intake, Transmission outtake, DoubleSolenoid pivotPiston) {
        this.intake = intake;
        this.outtake = outtake;
        pivot = pivotPiston;
    }

    @Override
    public void update() {
        cargoInIntake = cargoSensor.getVoltage() < Constants.kVoltageThreshold;
        cargoInCarriage = lastCargoInIntake && cargoInIntake;
        switch (currentIntakeState) {
            case WAITING:
                setIntakeDown(false);
                setIntakeSpeed(0);
                break;
            case INTAKING:
                setIntakeDown(true);
                setIntakeSpeed(1);
                break;
            case RAISING:
                setIntakeDown(false);
                setIntakeSpeed(0);
                break;
            case TRANSFERRING:
                setIntakeSpeed(1);
                setOuttakeSpeed(1);
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
        pivot.set(pivotState);
        intake.set(intakeSpeed, intakeControlMode);
        outtake.set(outtakeSpeed, outtakeControlMode);
        lastCargoInIntake = cargoInIntake;
    }

    /**
     * @param desiredSpeed the desiredSpeed to set
     */
    public void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }
    public void setOuttakeSpeed(double speed) {
        this.outtakeSpeed = speed;
    }

    public void setIntakeDown(boolean down) {
        if(down) {
            pivotState = Constants.kIntakeDown;
        }
        else {
            pivotState = Constants.kIntakeUp;
        }
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
        SmartDashboard.putNumber("Intake current", intake.getOutputCurrent(0));
        SmartDashboard.putNumber("Intake speed", intakeSpeed);
        SmartDashboard.putString("Intake state", getIntakeState().toString());
        SmartDashboard.putNumber("intake sensor voltage", cargoSensor.getVoltage());
    }

    @Override
    public void resetSensors() {
        setIntakeState(IntakeState.WAITING);
    }

    public boolean getCargoInIntake() {
        return cargoInIntake;
    }

    public boolean getCargoInCarriage() {
        return cargoInCarriage;
    }
}