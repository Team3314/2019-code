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
        MANUAL
    }

    private ElevatorControlMode currentElevatorMode = ElevatorControlMode.MOTION_MAGIC;
    private SpeedControllerMode controlMode;

    public void update(){
        switch(currentElevatorMode) {
            case MOTION_MAGIC:
                controlMode = SpeedControllerMode.kMotionMagic;
                break;
            case MANUAL:
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
        }
        transmission.set(demand, controlMode);
    }

    public Elevator(EncoderTransmission transmission) {
        super(transmission);

        setNeutralMode(IdleMode.kBrake);

    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Elevator Current", transmission.getOutputCurrent(0));
        SmartDashboard.putNumber("Elevator Voltage", transmission.getOutputVoltage());
        SmartDashboard.putNumber("Desired Elevator Position", demand);
        SmartDashboard.putNumber("Current Elevator Position ticks", transmission.getPosition());
        SmartDashboard.putNumber("Current Elevator Position Inches", getHeight());
        SmartDashboard.putString("Elevator Control Mode", getElevatorState().toString());
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
        return getPosition() * Constants.kElevatorTicksToInches;
    }

    public void setElevatorState(ElevatorControlMode mode) {
        if(currentElevatorMode != mode) {
            switch(mode) {
                case MOTION_MAGIC:
                    set(getPosition());
                    break;
                case MANUAL:
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