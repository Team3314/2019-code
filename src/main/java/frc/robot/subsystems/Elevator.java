package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.infrastructure.Lift;
import frc.robot.infrastructure.SensorTransmission;
import frc.robot.infrastructure.SpeedControllerMode;

public class Elevator extends Lift implements Subsystem {

    public enum ElevatorStateMachine {
        MOTION_MAGIC,
        MANUAL
    }

    private ElevatorStateMachine currentElevatorMode = ElevatorStateMachine.MOTION_MAGIC;
    private SpeedControllerMode controlMode;
    private double demand;

    private double desiredPosition, manualCommand;

    public void update(){
        switch(currentElevatorMode) {
            case MOTION_MAGIC:
                controlMode = SpeedControllerMode.kMotionMagic;
                demand = desiredPosition;
                break;
            case MANUAL:
                controlMode = SpeedControllerMode.kDutyCycle;
                demand = manualCommand;
                break;
        }
        transmission.set(demand, controlMode);
    }

    public Elevator(SensorTransmission transmission) {
        super(transmission);

        /*mMaster = new WPI_TalonSRX(6);
        mMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kCANTimeout);
    	mMaster.setSensorPhase(false);
    	mMaster.setInverted(false);
    	mMaster.configVoltageCompSaturation(Constants.kElevatorVoltageScale, Constants.kCANTimeout);
    	mMaster.enableVoltageCompensation(true);
    	mMaster.configOpenloopRamp(Constants.kElevatorOpenLoopRampRate, Constants.kCANTimeout);
    	mMaster.configNeutralDeadband(Constants.kElevatorDeadband, Constants.kCANTimeout);
    	mMaster.configContinuousCurrentLimit(Constants.kElevatorContinuousCurrentLimit, Constants.kCANTimeout);
    	mMaster.configPeakCurrentLimit(Constants.kElevatorPeakCurrentLimit, Constants.kCANTimeout);
    	mMaster.configPeakCurrentDuration(Constants.kElevatorCurrentDuration, Constants.kCANTimeout);
    	mMaster.enableCurrentLimit(true);
    	mMaster.configClosedloopRamp(Constants.kElevatorClosedLoopRampTime, Constants.kCANTimeout);
		mMaster.configForwardSoftLimitThreshold(Constants.kMaxElevatorPosition, Constants.kCANTimeout);
		mMaster.configReverseSoftLimitThreshold(Constants.kMinElevatorPosition, Constants.kCANTimeout);
		mMaster.configForwardSoftLimitEnable(true, Constants.kCANTimeout);
        mMaster.configReverseSoftLimitEnable(true, Constants.kCANTimeout);
        //TODO add limit switches

        mMaster.config_kP(Constants.kElevatorSlot, Constants.kElevator_kP, Constants.kCANTimeout);
        mMaster.config_kI(Constants.kElevatorSlot, Constants.kElevator_kI, Constants.kCANTimeout);
        mMaster.config_kD(Constants.kElevatorSlot, Constants.kElevator_kD, Constants.kCANTimeout);
        mMaster.config_kF(Constants.kElevatorSlot, Constants.kElevator_kF, Constants.kCANTimeout);

        mMaster.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kCANTimeout);
        mMaster.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kCANTimeout);*/

        setNeutralMode(NeutralMode.Brake);

    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Elevator Current", transmission.getOutputCurrent(0));
        SmartDashboard.putNumber("Elevator Voltage", transmission.getOutputVoltage());
        SmartDashboard.putNumber("Desired Elevator Position", desiredPosition);
        SmartDashboard.putNumber("Current Elevator Position ticks", transmission.getPosition());
        SmartDashboard.putString("Elevator Control Mode", getElevatorState().toString());
    }   


    public void resetSensors(){
        
    } 

    public void setDesiredPosition(double position) {
        desiredPosition = position;
    }

    public void setManualCommand(double command) {
        manualCommand = command;
    }

    private void setNeutralMode(NeutralMode mode) {
        setNeutralMode(mode);
    }

    public void setElevatorState(ElevatorStateMachine mode) {
        currentElevatorMode = mode;
    }

    public ElevatorStateMachine getElevatorState() {
		return currentElevatorMode;
	}

}