package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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