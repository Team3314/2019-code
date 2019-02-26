package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.infrastructure.IdleMode;
import frc.robot.infrastructure.Lift;
import frc.robot.Constants;
import frc.robot.infrastructure.EncoderTransmission;
import frc.robot.infrastructure.SpeedControllerMode;

public class Elevator extends Lift implements Subsystem {

    public enum ElevatorControlMode {
        MOTION_MAGIC,
        MANUAL,
        HOMING
    }

    private ElevatorControlMode currentElevatorMode = ElevatorControlMode.MOTION_MAGIC;
    private SpeedControllerMode controlMode;

    private boolean reverseLimit, lastReverseLimit;
    private boolean homed = false;

    public void update(){
        reverseLimit = transmission.getMotor(1).getReverseLimit();
        if(reverseLimit) {
            if(!lastReverseLimit)
                transmission.reset();
            if(demand < 0) {
                demand = 0;
            }
            homed = true;
        }
        switch(currentElevatorMode) {
            case MOTION_MAGIC:
                controlMode = SpeedControllerMode.kMotionMagic;
                break;
            case MANUAL:
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case HOMING:
                controlMode = SpeedControllerMode.kDutyCycle;
                set(-.1);
                if(transmission.getOutputCurrent(0) > 1 && transmission.getVelocity() < 100) {
                    homed = true;
                }
                if(homed)
                    currentElevatorMode = ElevatorControlMode.MOTION_MAGIC;
                break;
        }
        transmission.set(demand, controlMode);
        lastReverseLimit = reverseLimit;
    }

    public Elevator(EncoderTransmission transmission) {
        super(transmission);

        setNeutralMode(IdleMode.kBrake);

    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Elevator Master Current", transmission.getOutputCurrent(0));
        SmartDashboard.putNumber("Elevator Slave Current", transmission.getOutputCurrent(1));
        SmartDashboard.putNumber("Elevator Master Voltage", transmission.getOutputVoltage());
        SmartDashboard.putNumber("Desired Elevator Position", demand);
        SmartDashboard.putNumber("Current Elevator Position ticks", transmission.getPosition());
        SmartDashboard.putNumber("Current Elevator Position Inches", getHeight());
        SmartDashboard.putString("Elevator Control Mode", getElevatorState().toString());
        SmartDashboard.putBoolean("Reverse limit", transmission.getMotor(1).getReverseLimit());
    }   


    public void resetSensors(){
        transmission.reset();
    } 
    private void setNeutralMode(IdleMode mode) {
        transmission.setIdleMode(mode);
    }

    public double getPosition() {
        return transmission.getPosition();
    }

    public double getHeight() {
        return getPosition() * Constants.kElevatorInchesPerTick;
    }

    public boolean getHomed() {
        return homed;
    }

    public boolean isHoming() {
        return currentElevatorMode == ElevatorControlMode.HOMING;
    }

    public void setElevatorState(ElevatorControlMode mode) {
        if(currentElevatorMode != mode) {
            switch(mode) {
                case MOTION_MAGIC:
                    set(getPosition());
                    break;
                case MANUAL:
                    break;
                case HOMING:
                    break;
            }
            currentElevatorMode = mode;
        }
    }
    public ElevatorControlMode getElevatorState() {
		return currentElevatorMode;
    }
    public boolean inPosition() {
        return Math.abs(transmission.getPosition() - demand) <= Constants.kElevatorTolerance;
    }

}