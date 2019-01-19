package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.CustomPIDOutput;
import frc.robot.infrastructure.Drivetrain;
import frc.robot.infrastructure.IdleMode;
import frc.robot.infrastructure.SensorTransmission;
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

    public enum driveMode {
		IDLE,
		OPEN_LOOP,
        GYROLOCK,
        POSITION,
        VELOCITY,
        VISION_CONTROL, 
        MOTION_PROFILE
	}

    //Control Modes
    private driveMode currentDriveMode = driveMode.OPEN_LOOP;
    SpeedControllerMode controlMode = SpeedControllerMode.kIdle;
    
    //Hardware states
    private boolean mIsHighGear;
    private IdleMode idleMode;
    private double rawLeftSpeed, rawRightSpeed, desiredLeftSpeed, desiredRightSpeed, desiredAngle, desiredPosition,
    leftDrivePositionTicks, rightDrivePositionTicks, leftDriveSpeedTicks, rightDriveSpeedTicks, leftStickInput, rightStickInput;
    
    private double leftDrivePositionInches, rightDrivePositionInches, leftDriveSpeedRPM, rightDriveSpeedRPM;
    private PIDController gyroControl;
    private CustomPIDOutput gyroPIDOutput;

    private Camera camera;

    public Drive(SensorTransmission left, SensorTransmission right, AHRS gyro){
        super(left, right);
        camera = Camera.getInstance();
    	 
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
                rawLeftSpeed = 0;
                rawRightSpeed = 0;
                break;
            case OPEN_LOOP:
                rawLeftSpeed = leftStickInput;
                rawRightSpeed = rightStickInput;
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case GYROLOCK:
                rawLeftSpeed = desiredLeftSpeed + gyroPIDOutput.getOutput();
                rawRightSpeed = desiredRightSpeed - gyroPIDOutput.getOutput();
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case VISION_CONTROL:
                rawLeftSpeed = desiredLeftSpeed + camera.getCorrection();
                rawRightSpeed = desiredRightSpeed - camera.getCorrection();
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case POSITION:
                rawLeftSpeed = desiredLeftSpeed;
                rawRightSpeed = desiredRightSpeed;
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kPosition;
                break;
            case VELOCITY:
                rawLeftSpeed = desiredLeftSpeed;
                rawRightSpeed = desiredRightSpeed;
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case MOTION_PROFILE:
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kVelocity;
                rawLeftSpeed = desiredLeftSpeed;
                rawRightSpeed = desiredRightSpeed;
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
    public void setStickInputs(double leftInput, double rightInput) {
    	leftStickInput = leftInput * Math.abs(leftInput);
    	rightStickInput = rightInput * Math.abs(rightInput);
    }
    
    public void setDesiredAngle(double angle) {
    	desiredAngle = angle;
    }
    
    public void setDesiredPosition(double d) {
    	desiredPosition = d * Constants.kFeetToEncoderCodes;
    }
    
    public double getDesiredPosition() {
    	return desiredPosition;
    }
    
    public double getDesiredAngle() {
    	return desiredAngle;
    }
    
    public double getAveragePositionTicks() {
    	return (leftDrivePositionTicks + rightDrivePositionTicks) / 2;
    }
    
    public double getAverageDistance() {
    	return getAveragePositionTicks() / Constants.kFeetToEncoderCodes;
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
    
    public void setDesiredSpeed(double speed) {
    	desiredLeftSpeed = speed;
    	desiredRightSpeed = speed;
    }
    
    public void setDesiredSpeed(double leftSpeed, double rightSpeed) {
    	desiredLeftSpeed = leftSpeed;
    	desiredRightSpeed = rightSpeed;
    }
    
    public void setDriveMode(driveMode mode) {
    	if(mode == driveMode.GYROLOCK) {
    		gyroControl.enable();
			setDesiredAngle(getAngle());
    	}
    	else {
    		gyroControl.disable();
    	}
    	if(mode == driveMode.MOTION_PROFILE) {

    	}
    	currentDriveMode = mode;
    	
    }
    
    public void setHighGear(boolean highGear) {
    	mIsHighGear = highGear;
    }
    
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Left Encoder Ticks", leftDrivePositionTicks);
    	SmartDashboard.putNumber("Right Encoder Ticks", rightDrivePositionTicks);
    	SmartDashboard.putNumber("Left Encoder Position", leftDrive.getPosition());
    	SmartDashboard.putNumber("Right Encoder Position", rightDrive.getPosition());
    	SmartDashboard.putNumber("Left Encoder Speed", leftDrive.getVelocity());
    	SmartDashboard.putNumber("Right Encoder Speed", rightDrive.getVelocity());
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
    	SmartDashboard.putNumber("Desired Left Speed", desiredLeftSpeed);
    	SmartDashboard.putNumber("Desired Right Speed", desiredRightSpeed);
    	SmartDashboard.putNumber("Left Voltage", leftDrive.getOutputVoltage());
    	SmartDashboard.putNumber("Right Voltage", rightDrive.getOutputVoltage());
    }
    
    private void updateSpeedAndPosition() {
    	leftDrivePositionTicks = leftDrive.getPosition();
    	rightDrivePositionTicks = rightDrive.getPosition();
    	leftDrivePositionInches = (double) leftDrivePositionTicks / Constants.kDriveEncoderCodesPerRev  * Constants.kRevToInConvFactor;
    	rightDrivePositionInches =  (double) rightDrivePositionTicks / Constants.kDriveEncoderCodesPerRev * Constants.kRevToInConvFactor;
    	leftDriveSpeedRPM = leftDriveSpeedTicks * (600.0/ Constants.kDriveEncoderCodesPerRev);
    	rightDriveSpeedRPM =  rightDriveSpeedTicks * (600.0/ Constants.kDriveEncoderCodesPerRev);
    }
  
    public void resetDriveEncoders() {
    }
    
    public void resetSensors() {
    	navx.reset();
    	resetDriveEncoders();
    }
    
    public boolean gyroInPosition() {
    	return gyroControl.onTarget();
    }
}