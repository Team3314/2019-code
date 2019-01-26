package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.CustomPIDOutput;
import frc.robot.Robot;
import frc.robot.infrastructure.Drivetrain;
import frc.robot.infrastructure.IdleMode;
import frc.robot.infrastructure.EncoderTransmission;
import frc.robot.infrastructure.SpeedControllerMode;

/*
SPARK FF TO DUTY CYCLE CONVERSION
DUTY CYCLE = FF * 300
*/

public class Drive extends Drivetrain implements Subsystem {

//Hardware

    private AHRS navx;

    private DoubleSolenoid shifter;
    
    private PowerDistributionPanel pdp;

    public enum DriveMode {
		IDLE,
		OPEN_LOOP,
        GYROLOCK,
        POSITION,
        VELOCITY,
        VISION_CONTROL, 
        MOTION_PROFILE
	}

    //Control Modes
    private DriveMode currentDriveMode = DriveMode.OPEN_LOOP;
    SpeedControllerMode controlMode = SpeedControllerMode.kIdle;
    
    //Hardware states
    private boolean mIsHighGear;
    private IdleMode idleMode;
    private double rawLeftSpeed, rawRightSpeed, desiredAngle;
    
    private double leftDrivePositionInches, rightDrivePositionInches, leftDrivePositionTicks, rightDrivePositionTicks, leftDriveSpeedRPM, rightDriveSpeedRPM, 
        leftDriveSpeedInches, rightDriveSpeedInches;
    private PIDController gyroControl;
    private CustomPIDOutput gyroPIDOutput;

    private Camera camera;

    public Drive(EncoderTransmission left, EncoderTransmission right, AHRS gyro){
        super(left, right);
        camera = Robot.camera;
    	 
		//Hardware
    	pdp  = new PowerDistributionPanel(0);
    	shifter = new DoubleSolenoid(0, 1);
        navx = gyro;

        gyroPIDOutput = new CustomPIDOutput();
    	gyroControl = new PIDController(Constants.kGyroLock_kP, Constants.kGyroLock_kI, Constants.kGyroLock_kD,
    		Constants.kGyroLock_kF, navx, gyroPIDOutput);
		//Sets the PID controller to treat 180 and -180 to be the same point, 
		//so that when turning the robot takes the shortest path instead of going the long way around
		//Effectively changes PID input from a line to a circle
		gyroControl.setOutputRange(-Constants.kGyroOutputRange, Constants.kGyroOutputRange);		// Limits speed of turn to prevent overshoot
		gyroControl.setAbsoluteTolerance(Constants.kAbsoluteGyroTolerance);

    }

    public void update(){
        if(mIsHighGear) {
    		shifter.set(Constants.kHighGear);
    	}
    	else {
    		shifter.set(Constants.kLowGear);
        }
        updateSpeedAndPosition();
        switch(currentDriveMode) {
            case IDLE:
                controlMode = SpeedControllerMode.kDutyCycle;
                setIdleMode(IdleMode.kBrake);
                rawLeftSpeed = 0;
                rawRightSpeed = 0;
                break;
            case OPEN_LOOP:
                rawLeftSpeed = leftDemand * Math.abs(leftDemand);
                rawRightSpeed = rightDemand * Math.abs(rightDemand);
                setIdleMode(IdleMode.kCoast);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case GYROLOCK:
                rawLeftSpeed = leftDemand + gyroPIDOutput.getOutput();
                rawRightSpeed = rightDemand - gyroPIDOutput.getOutput();
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case VISION_CONTROL:
                rawLeftSpeed = leftDemand + camera.getCorrection();
                rawRightSpeed = rightDemand - camera.getCorrection();
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case POSITION:
                rawLeftSpeed = leftDrive.getPIDOutput();
                rawRightSpeed = rightDrive.getPIDOutput();
                leftDrive.setSetpoint(leftDemand);
                rightDrive.setSetpoint(rightDemand);
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case VELOCITY:
                rawLeftSpeed = leftDemand;
                rawRightSpeed = rightDemand;
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kVelocity;
                break;
            case MOTION_PROFILE:
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kVelocity;
                rawLeftSpeed = leftDemand;
                rawRightSpeed = rightDemand;
                break;
        }

        rightDrive.set(rawRightSpeed, controlMode);
        leftDrive.set(rawLeftSpeed, controlMode);

    }

    public void setIdleMode(IdleMode mode) {
    	if(idleMode != mode) {
            idleMode = mode;

            leftDrive.setIdleMode(mode);
            rightDrive.setIdleMode(mode);
    	}
    }
    
    public void setDesiredAngle(double angle) {
    	desiredAngle = angle;
    }
    
    public double getDesiredAngle() {
    	return desiredAngle;
    }
    
    public double getAngle() {
    	return navx.getYaw();
    }

    public double getLeftPositionTicks() {
        return leftDrivePositionTicks;
    }

    public double getRightPositionTicks() {
        return rightDrivePositionTicks;
    }
    
    public double getLeftPosition() {
    	return leftDrivePositionInches;
    }
    
    public double getRightPosition() {
    	return rightDrivePositionInches;
    }
    
    public double getAveragePosition() {
    	return (leftDrivePositionInches+rightDrivePositionInches)/2;
    }
    
    public void setDriveMode(DriveMode mode) {
        if(mode != currentDriveMode) {
            if(mode == DriveMode.GYROLOCK) {
                gyroControl.enable();
                setDesiredAngle(getAngle());
            }
            else {
                gyroControl.disable();
            }
            if(mode == DriveMode.POSITION) {
                leftDrive.enablePID();
                rightDrive.enablePID();
                leftDrive.setPIDSourceType(PIDSourceType.kDisplacement);
                rightDrive.setPIDSourceType(PIDSourceType.kDisplacement);
            }
            else if(mode == DriveMode.VELOCITY) {
                leftDrive.enablePID();
                rightDrive.enablePID();
                leftDrive.setPIDSourceType(PIDSourceType.kRate);
                rightDrive.setPIDSourceType(PIDSourceType.kRate);
            }
            else {
                leftDrive.disablePID();
                rightDrive.disablePID();
            }

            if(mode == DriveMode.MOTION_PROFILE) {

            }
            currentDriveMode = mode;
        }
    }

    public void updateSpeedAndPosition() {
        leftDrivePositionTicks = leftDrive.getPosition();
        rightDrivePositionTicks = rightDrive.getPosition();
        leftDrivePositionInches = leftDrivePositionTicks * Constants.kRevToInConvFactor;
        rightDrivePositionInches = rightDrivePositionTicks * Constants.kRevToInConvFactor;
        leftDriveSpeedRPM = leftDrive.getVelocity();
        rightDriveSpeedRPM = rightDrive.getVelocity();
        leftDriveSpeedInches = leftDriveSpeedRPM/60 * Constants.kRevToInConvFactor;
        rightDriveSpeedInches = rightDriveSpeedRPM/60 * Constants.kRevToInConvFactor;

    }
    
    public void setHighGear(boolean highGear) {
    	mIsHighGear = highGear;
    }
    
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Left Encoder Position Ticks", leftDrivePositionTicks);
    	SmartDashboard.putNumber("Right Encoder Position Ticks", rightDrivePositionTicks);
    	SmartDashboard.putNumber("Left Encoder Inches", leftDrivePositionInches);
    	SmartDashboard.putNumber("Right Encoder Inches", rightDrivePositionInches);
    	SmartDashboard.putNumber("Left Encoder Speed RPM", leftDriveSpeedRPM);
        SmartDashboard.putNumber("Right Encoder Speed RPM", rightDriveSpeedRPM);
        SmartDashboard.putNumber("Left Encoder Speed Inches/Second", leftDriveSpeedInches);
        SmartDashboard.putNumber("Left Encoder Speed Inches/Second", rightDriveSpeedInches);
    	SmartDashboard.putNumber("Left Master Current", leftDrive.getOutputCurrent(0));
    	SmartDashboard.putNumber("Left Slave 1 Current", leftDrive.getOutputCurrent(1));
    	SmartDashboard.putNumber("Left Slave 2 Current", leftDrive.getOutputCurrent(2));
    	SmartDashboard.putNumber("Right Master Current", rightDrive.getOutputCurrent(0));
    	SmartDashboard.putNumber("Right Slave 1 Current", rightDrive.getOutputCurrent(1));
    	SmartDashboard.putNumber("Right Slave 2 Current", rightDrive.getOutputCurrent(2));
    	SmartDashboard.putString("Drive Mode", String.valueOf(currentDriveMode));
    	SmartDashboard.putString("Neutral Mode", String.valueOf(idleMode));
    	SmartDashboard.putNumber("Raw Left Speed", rawLeftSpeed);
    	SmartDashboard.putNumber("Raw Right Speed", rawRightSpeed);
    	SmartDashboard.putNumber("Desired Angle", desiredAngle);
    	SmartDashboard.putNumber("Current angle", navx.getAngle());
    	SmartDashboard.putNumber("Gyro adjustment", gyroPIDOutput.getOutput());
    	SmartDashboard.putNumber("Left Demand", leftDemand);
    	SmartDashboard.putNumber("Right Demand", rightDemand);
    	SmartDashboard.putNumber("Left Voltage", leftDrive.getOutputVoltage());
    	SmartDashboard.putNumber("Right Voltage", rightDrive.getOutputVoltage());
    }
  
    public void resetDriveEncoders() {
        leftDrive.reset();
        rightDrive.reset();
    }
    
    public void resetSensors() {
    	navx.reset();
    	resetDriveEncoders();
    }
    
    public boolean gyroInPosition() {
    	return gyroControl.onTarget();
    }
    public boolean collision(){
        return 20 < Math.round(100* navx.getWorldLinearAccelX());
    }
}