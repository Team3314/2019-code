package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class CargoIntake implements Subsystem {

    public enum IntakeState {
        //UNJAMMING,
        HOLDING,
        INTAKING,
        RELEASING
    }

    private static CargoIntake mInstance = new CargoIntake();
    
    private WPI_TalonSRX mIntakeMotor;
    private double desiredSpeed;
    private IntakeState currentState = IntakeState.HOLDING;

    public static CargoIntake getInstance() {
        return mInstance;
    }

    private CargoIntake() {
        mIntakeMotor = new WPI_TalonSRX(63);
        mIntakeMotor.configContinuousCurrentLimit(Constants.kIntakeContinuousCurrentLimit, 0);
        mIntakeMotor.configPeakCurrentLimit(Constants.kIntakePeakCurrentLimit, 0);
        mIntakeMotor.configPeakCurrentDuration(Constants.kIntakeCurrentDuration, 0);
        mIntakeMotor.enableCurrentLimit(true);
        mIntakeMotor.setInverted(false);
        mIntakeMotor.configPeakOutputForward(1, 0);
        mIntakeMotor.configPeakOutputReverse(-1, 0);
        mIntakeMotor.configForwardSoftLimitEnable(false, 0);
        mIntakeMotor.configReverseSoftLimitEnable(false, 0);
        mIntakeMotor.configVoltageCompSaturation(12.0, 0);
        mIntakeMotor.enableVoltageCompensation(true);
    }

    @Override
    public void update() {
        switch (currentState) {
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
        mIntakeMotor.set(ControlMode.PercentOutput, desiredSpeed);
    }

    /**
     * @param desiredSpeed the desiredSpeed to set
     */
    public void setDesiredSpeed(double speed) {
        desiredSpeed = speed;
    }

    /**
     * @param currentState the currentState to set
     */
    public void setCurrentState(IntakeState desiredState) {
        currentState = desiredState;
    }

    /**
     * @return the currentState
     */
    public IntakeState getCurrentState() {
        return currentState;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Intake current", mIntakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake speed", desiredSpeed);
        SmartDashboard.putString("Intake state", getCurrentState().toString());
    }

    @Override
    public void resetSensors() {
        setCurrentState(IntakeState.HOLDING);
    }

}