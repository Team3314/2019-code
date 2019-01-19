package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.CustomPIDOutput;

/*
SPARK FF TO DUTY CYCLE CONVERSION
DUTY CYCLE = FF * 300
*/

public class Drive implements Subsystem {

//Hardware
    private AHRS navx;

    private CANSparkMax mLeftMaster, mLeftSlave1, mLeftSlave2, mRightMaster, mRightSlave1, mRightSlave2;

    private DoubleSolenoid shifter;
    
    private PowerDistributionPanel pdp;

    public enum driveMode {
		IDLE,
		OPEN_LOOP,
		GYROLOCK,
        VELOCITY,
        VISION_CONTROL, 
        MOTION_PROFILE
	}

    private static Drive mInstance = new Drive();

    //Control Modes
    private driveMode currentDriveMode = driveMode.OPEN_LOOP;
    ControlType controlMode = ControlType.kDutyCycle;
    
    //Hardware states
    private boolean mIsHighGear;
    private IdleMode idleMode;
    private double rawLeftSpeed, rawRightSpeed, leftStickInput, rightStickInput, desiredLeftSpeed, desiredRightSpeed, desiredAngle, desiredPosition,
    leftDrivePositionTicks, rightDrivePositionTicks, leftDriveSpeedTicks, rightDriveSpeedTicks;
    
    private double leftDrivePositionInches, rightDrivePositionInches, leftDriveSpeedRPM, rightDriveSpeedRPM;

    //PID
	private CustomPIDOutput gyroPIDOutput;
    private PIDController gyroControl;
    private CANPIDController leftDrive, rightDrive;
    private CANEncoder leftEncoder, rightEncoder;
    
    private Camera camera = Camera.getInstance();


    /**
     * @return the mInstance
     */
    public static Drive getInstance() {
        return mInstance;
    }

    private Drive(){
        camera = Camera.getInstance();
    	 
		//Hardware
    	pdp  = new PowerDistributionPanel(0);
    	shifter = new DoubleSolenoid(0, 1);
        navx = new AHRS(SerialPort.Port.kMXP);

        mLeftMaster = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftMaster.setCANTimeout(Constants.kCANTimeout);
        mLeftMaster.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);

        leftDrive = new CANPIDController(mLeftMaster);
        
        leftDrive.setP(Constants.kVelocity_kP);
        leftDrive.setI(Constants.kVelocity_kI);
        leftDrive.setD(Constants.kVelocity_kD);
        leftDrive.setFF(Constants.kVelocity_kF);
        leftDrive.setIZone(Constants.kVelocity_kIZone);

        leftEncoder = new CANEncoder(mLeftMaster);

        mLeftSlave1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave1.setCANTimeout(Constants.kCANTimeout);
        mLeftSlave1.follow(mLeftMaster);
        mLeftSlave1.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);

        mLeftSlave2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave2.setCANTimeout(Constants.kCANTimeout);
        mLeftSlave2.follow(mLeftMaster);
        mLeftSlave2.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);

        mRightMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMaster.setCANTimeout(Constants.kCANTimeout);
        mRightMaster.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);

        rightDrive = new CANPIDController(mRightMaster);

        rightDrive.setP(Constants.kVelocity_kP);
        rightDrive.setI(Constants.kVelocity_kI);
        rightDrive.setD(Constants.kVelocity_kD);
        rightDrive.setFF(Constants.kVelocity_kF);
        rightDrive.setIZone(Constants.kVelocity_kIZone);
        
        rightEncoder = new CANEncoder(mRightMaster);

        mRightSlave1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave1.setCANTimeout(Constants.kCANTimeout);
        mRightSlave1.follow(mRightMaster);
        mRightSlave1.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);

        mRightSlave2 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave2.setCANTimeout(Constants.kCANTimeout);
        mRightSlave2.follow(mRightMaster);
        mRightSlave2.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);

        shifter = new DoubleSolenoid(0, 1);

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
                controlMode = ControlType.kDutyCycle;
                rawLeftSpeed = 0;
                rawRightSpeed = 0;
                break;
            case OPEN_LOOP:
                rawLeftSpeed = desiredLeftSpeed;
                rawRightSpeed = desiredRightSpeed;
                setIdleMode(IdleMode.kBrake);
                controlMode = ControlType.kDutyCycle;
                break;
            case GYROLOCK:
                rawLeftSpeed = desiredLeftSpeed + gyroPIDOutput.getOutput();
                rawRightSpeed = desiredRightSpeed - gyroPIDOutput.getOutput();
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                controlMode = ControlType.kDutyCycle;
                break;
            case VISION_CONTROL:
                rawLeftSpeed = desiredLeftSpeed + camera.getCorrection();
                rawRightSpeed = desiredRightSpeed - camera.getCorrection();
                setIdleMode(IdleMode.kBrake);
                controlMode = ControlType.kDutyCycle;
                break;
            case VELOCITY:
                rawLeftSpeed = desiredLeftSpeed;
                rawRightSpeed = desiredRightSpeed;
                setIdleMode(IdleMode.kBrake);
                controlMode = ControlType.kDutyCycle;
                break;
            case MOTION_PROFILE:
                setIdleMode(IdleMode.kBrake);
                controlMode = ControlType.kVelocity;
                rawLeftSpeed = desiredLeftSpeed;
                rawRightSpeed = desiredRightSpeed;
                break;
        }

        rightDrive.setReference(rawRightSpeed, controlMode);
        leftDrive.setReference(rawLeftSpeed, controlMode);

    }

    public void setIdleMode(IdleMode mode) {
    	if(idleMode != mode) {
            idleMode = mode;

            mLeftMaster.setIdleMode(mode);
            mLeftSlave1.setIdleMode(mode);
            mLeftSlave2.setIdleMode(mode);
            mRightMaster.setIdleMode(mode);
            mRightSlave1.setIdleMode(mode);
            mRightSlave2.setIdleMode(mode);
    	}
    }
    public void setStickInputs(double leftInput, double rightInput) {
    	desiredLeftSpeed =Math.copySign(leftInput * leftInput, leftInput);
    	desiredRightSpeed = Math.copySign(rightInput * rightInput, rightInput);
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
    	else {
    		setRampRate(Constants.kDriveOpenLoopRampRate);
    	}
    	currentDriveMode = mode;
    	
    }
    
    public void setHighGear(boolean highGear) {
    	mIsHighGear = highGear;
    }
    
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Left Encoder Ticks", leftDrivePositionTicks);
    	SmartDashboard.putNumber("Right Encoder Ticks", rightDrivePositionTicks);
    	SmartDashboard.putNumber("Left Encoder Position", leftDrivePositionInches);
    	SmartDashboard.putNumber("Right Encoder Position", rightDrivePositionInches);
    	SmartDashboard.putNumber("Left Encoder Speed", leftDriveSpeedRPM);
    	SmartDashboard.putNumber("Right Encoder Speed", rightDriveSpeedRPM);
    	SmartDashboard.putNumber("Left Master Current", mLeftMaster.getOutputCurrent());
    	SmartDashboard.putNumber("Left Slave 1 Current", mLeftSlave1.getOutputCurrent());
    	SmartDashboard.putNumber("Left Slave 2 Current", mLeftSlave2.getOutputCurrent());
    	SmartDashboard.putNumber("Right Master Current", mRightMaster.getOutputCurrent());
    	SmartDashboard.putNumber("Right Slave 1 Current", mRightSlave1.getOutputCurrent());
    	SmartDashboard.putNumber("Right Slave 2 Current", mRightSlave2.getOutputCurrent());
    	SmartDashboard.putString("Drive Mode", String.valueOf(currentDriveMode));
    	SmartDashboard.putString("Neutral Mode", String.valueOf(idleMode));
    	SmartDashboard.putNumber("Raw Left Speed", rawLeftSpeed);
    	SmartDashboard.putNumber("Raw Right Speed", rawRightSpeed);
    	SmartDashboard.putNumber("Desired Angle", desiredAngle);
    	SmartDashboard.putNumber("Current angle", navx.getAngle());
    	SmartDashboard.putNumber("Gyro adjustment", gyroPIDOutput.getOutput());
    	SmartDashboard.putNumber("Desired Left Speed", desiredLeftSpeed);
    	SmartDashboard.putNumber("Desired Right Speed", desiredRightSpeed);
    	SmartDashboard.putNumber("Left Voltage", mLeftMaster.getAppliedOutput());
    	SmartDashboard.putNumber("Right Voltage", mRightMaster.getAppliedOutput());
    }
    
    private void updateSpeedAndPosition() {
    	leftDrivePositionTicks = leftEncoder.getPosition();
    	rightDrivePositionTicks = rightEncoder.getPosition();
    	leftDrivePositionInches = (double) leftDrivePositionTicks / Constants.kDriveEncoderCodesPerRev  * Constants.kRevToInConvFactor;
    	rightDrivePositionInches =  (double) rightDrivePositionTicks / Constants.kDriveEncoderCodesPerRev * Constants.kRevToInConvFactor;
    	leftDriveSpeedTicks = leftEncoder.getVelocity();
    	rightDriveSpeedTicks =  rightEncoder.getVelocity();
    	leftDriveSpeedRPM = leftDriveSpeedTicks * (600.0/ Constants.kDriveEncoderCodesPerRev);
    	rightDriveSpeedRPM =  rightDriveSpeedTicks * (600.0/ Constants.kDriveEncoderCodesPerRev);
    }
  
    public void resetDriveEncoders() {
    }
    
    public void resetSensors() {
    	navx.reset();
    	resetDriveEncoders();
    }
    
    public void setRampRate(double seconds) {
    	mLeftMaster.setRampRate(seconds);
    	mRightMaster.setRampRate(seconds);
    }
    
    public boolean gyroInPosition() {
    	return gyroControl.onTarget();
    }
}