package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HumanInput;
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
        STOP_DOWN,
        CLIMB,
        OVERRIDE
    }

    private Transmission intake, outtake;
    private DoubleSolenoid pivot;
    private Solenoid highPressure;

    private double intakeSpeed, outtakeSpeed;

    private Value pivotState = Constants.kIntakeUp;

    private boolean stopUp = false, high = false;

    private AnalogInput intakeCargoSensor = new AnalogInput(0), elevatorCargoSensor = new AnalogInput(1);
    private DigitalInput raisedSensor = new DigitalInput(4);
    private IntakeState currentIntakeState = IntakeState.WAITING;
    private Servo rightStop, leftStop;
    SpeedControllerMode intakeControlMode = SpeedControllerMode.kDutyCycle;
    SpeedControllerMode outtakeControlMode = SpeedControllerMode.kDutyCycle;

    public CargoIntake(Transmission intake, Transmission outtake, DoubleSolenoid pivotPiston, Servo rightStop, Servo leftStop, Solenoid highPressure) {
        this.intake = intake;
        this.outtake = outtake;
        this.leftStop = leftStop;
        this.rightStop = rightStop;
        pivot = pivotPiston;
        this.highPressure = highPressure;
    }

    @Override
    public void update() {
        stopUp = true;
        high = false;
        setIntakeSpeed(0);
        setOuttakeSpeed(0);
        switch (currentIntakeState) {
            case WAITING:
                setIntakeDown(false);
                setIntakeSpeed(0);
                setOuttakeSpeed(0);
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
                setOuttakeSpeed(.25);
                setIntakeDown(false);
                break;
            case PLACE:
                setOuttakeSpeed(1);
                break;
            case VOMIT:
                setIntakeSpeed(-1);
                break;
            case STOP_DOWN:
                stopUp = false;
                high = true;
                setIntakeDown(false);
                break;
            case CLIMB:
                stopUp = false;
                high = true;
                setIntakeDown(true);
                break;
            case OVERRIDE:
                setIntakeDown(true);
                setIntakeSpeed(1);
                break;
        }
        setStopUp(stopUp);
        highPressure.set(high);
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

    private void setStopUp(boolean up) {
        if(up) {
            rightStop.setAngle(Constants.kRightStopUpAngle);
            leftStop.setAngle(Constants.kLeftStopUpAngle);
        }
        else {
            rightStop.setAngle(Constants.kRightStopDownAngle);
            leftStop.setAngle(Constants.kLeftStopDownAngle);
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
        SmartDashboard.putNumber("Cargo Intake current", intake.getOutputCurrent(0));
        SmartDashboard.putNumber("Cargo Intake speed", intakeSpeed);
        SmartDashboard.putString("Cargo Intake state", getIntakeState().toString());
        SmartDashboard.putNumber("Cargo intake sensor voltage", intakeCargoSensor.getVoltage());
        SmartDashboard.putBoolean("Ball in Intake", getCargoInIntake());
        SmartDashboard.putBoolean("Ball in Carriage", getCargoInCarriage());
        SmartDashboard.putNumber("Right Stop Angle", rightStop.getAngle());
        SmartDashboard.putNumber("Left Stop Angle", leftStop.getAngle());
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