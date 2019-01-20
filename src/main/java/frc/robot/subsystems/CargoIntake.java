package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.infrastructure.SensorTransmission;
import frc.robot.infrastructure.SpeedControllerMode;

public class CargoIntake implements Subsystem {

    public enum IntakeStateMachine {
        HOLDING,
        INTAKING,
        RELEASING
    }

    private SensorTransmission intake;
    private double speed;

    private IntakeStateMachine currentIntakeMode = IntakeStateMachine.HOLDING;
    SpeedControllerMode controlMode = SpeedControllerMode.kDutyCycle;

    public CargoIntake(SensorTransmission transmission) {
        intake = transmission;
        /*mIntakeMotor = new WPI_TalonSRX(63);
        mIntakeMotor.configContinuousCurrentLimit(Constants.kIntakeContinuousCurrentLimit, Constants.kCANTimeout);
        mIntakeMotor.configPeakCurrentLimit(Constants.kIntakePeakCurrentLimit, Constants.kCANTimeout);
        mIntakeMotor.configPeakCurrentDuration(Constants.kIntakeCurrentDuration, Constants.kCANTimeout);
        mIntakeMotor.enableCurrentLimit(true);
        mIntakeMotor.setInverted(false);
        mIntakeMotor.configPeakOutputForward(1, Constants.kCANTimeout);
        mIntakeMotor.configPeakOutputReverse(-1, Constants.kCANTimeout);
        mIntakeMotor.configForwardSoftLimitEnable(false, Constants.kCANTimeout);
        mIntakeMotor.configReverseSoftLimitEnable(false, Constants.kCANTimeout);
        mIntakeMotor.configVoltageCompSaturation(12, Constants.kCANTimeout);
        mIntakeMotor.enableVoltageCompensation(true);*/
    }

    @Override
    public void update() {
        switch (currentIntakeMode) {
            case HOLDING:
                setDesiredSpeed(0);
                break;
            case INTAKING:
                setDesiredSpeed(1);
                break;
            case RELEASING:
                setDesiredSpeed(-1);
                break;
        }
        intake.set(speed, controlMode);
    }

    /**
     * @param desiredSpeed the desiredSpeed to set
     */
    public void setDesiredSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * @param currentState the currentState to set
     */
    public void setIntakeState(IntakeStateMachine mode) {
        currentIntakeMode = mode;
    }

    /**
     * @return the currentState
     */
    public IntakeStateMachine getIntakeState() {
        return currentIntakeMode;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Intake current", intake.getOutputCurrent(0));
        SmartDashboard.putNumber("Intake speed", speed);
        SmartDashboard.putString("Intake state", getIntakeState().toString());
    }

    @Override
    public void resetSensors() {
        setIntakeState(IntakeStateMachine.HOLDING);
    }

}