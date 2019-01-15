package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.CustomPIDOutput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonDrive implements Subsystem {
	
	public enum driveMode {
		IDLE,
		OPEN_LOOP,
		GYROLOCK,
		VELOCITY,
		VISION_CONTROL,
		MOTION_PROFILE,
	}
	
	private static TalonDrive mInstance = new TalonDrive();
	
	public static TalonDrive getInstance() {
		return mInstance;
	}
	
	//Control mode
	private driveMode currentDriveMode = driveMode.OPEN_LOOP;
	ControlMode controlMode = ControlMode.PercentOutput;
	
	//Hardware
	private WPI_TalonSRX mLeftMaster, mLeftSlave1, mLeftSlave2, mRightMaster, mRightSlave1, mRightSlave2;
	private DoubleSolenoid shifter;
	private PowerDistributionPanel pdp;
	private AHRS navx;
	
	private DigitalInput leftHighGearSensor, leftLowGearSensor, rightHighGearSensor, rightLowGearSensor;
	
	//Hardware states
	private boolean mIsHighGear;
    private NeutralMode neutralMode;
    
    //Data Logging
    public Camera camera;
    
    //PID
	private CustomPIDOutput gyroPIDOutput;
	private PIDController gyroControl;

    private double rawLeftSpeed, rawRightSpeed, desiredLeftSpeed, desiredRightSpeed, desiredAngle, desiredPosition;
    
    private int leftDrivePositionTicks, rightDrivePositionTicks, leftDriveSpeedTicks, rightDriveSpeedTicks;
    
    private double leftDrivePositionInches, rightDrivePositionInches, leftDriveSpeedRPM, rightDriveSpeedRPM;
    
    Compressor pcm1;
    
    public void update() {
    	if(mIsHighGear) {
    		shifter.set(Constants.kHighGear);
    	}
    	else {
    		shifter.set(Constants.kLowGear);
    	}
    	updateSpeedAndPosition();
    	switch(currentDriveMode) {
    		case IDLE:
    			controlMode = ControlMode.Disabled;
    			rawLeftSpeed = 0;
    			rawRightSpeed = 0;
    			break;
    		case OPEN_LOOP:
    			rawLeftSpeed = desiredLeftSpeed;
    			rawRightSpeed = desiredRightSpeed;
    			setNeutralMode(NeutralMode.Brake);
    			controlMode = ControlMode.PercentOutput;
    			break;
    		case GYROLOCK:
    			rawLeftSpeed = desiredLeftSpeed + gyroPIDOutput.getOutput();
    			rawRightSpeed = desiredRightSpeed - gyroPIDOutput.getOutput();
    			gyroControl.setSetpoint(desiredAngle);
    			setNeutralMode(NeutralMode.Brake);
    			controlMode = ControlMode.PercentOutput;
    			break;
    		case VISION_CONTROL:
    			rawLeftSpeed = desiredLeftSpeed + camera.getCorrection();
    			rawRightSpeed = desiredRightSpeed - camera.getCorrection();
    			setNeutralMode(NeutralMode.Brake);
    			controlMode = ControlMode.PercentOutput;
    			break;
    		case MOTION_PROFILE:
    			setNeutralMode(NeutralMode.Brake);
    			controlMode = ControlMode.Velocity;
    			rawLeftSpeed = desiredLeftSpeed;
    			rawRightSpeed = desiredRightSpeed;
    			break;
		default:
			break;
		}
		
    	mLeftMaster.set(controlMode, rawLeftSpeed);
    	mRightMaster.set(controlMode, rawRightSpeed);
  
    }

    private TalonDrive() {
    	camera = Camera.getInstance();
    	 
		//Hardware
    	pdp  = new PowerDistributionPanel(0);
    	shifter = new DoubleSolenoid(0, 1);
    	navx = new AHRS(SPI.Port.kMXP);
    	
    	//Gyro PID
    	gyroPIDOutput = new CustomPIDOutput();
    	gyroControl = new PIDController(Constants.kGyroLock_kP, Constants.kGyroLock_kI, Constants.kGyroLock_kD,
    		Constants.kGyroLock_kF, navx, gyroPIDOutput);
		//Sets the PID controller to treat 180 and -180 to be the same point, 
		//so that when turning the robot takes the shortest path instead of going the long way around
		//Effectively changes PID input from a line to a circle
		gyroControl.setOutputRange(-.5, .5);		// Limits speed of turn to prevent overshoot
		gyroControl.setAbsoluteTolerance(3);
    	
		//Talons
    	mRightMaster = new WPI_TalonSRX(7);
    	mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kCANTimeout);
    	mRightMaster.setSensorPhase(true);
    	mRightMaster.setInverted(true);
    	mRightMaster.configMotionProfileTrajectoryPeriod(Constants.kDriveMotionControlTrajectoryPeriod, Constants.kCANTimeout);
    	mRightMaster.changeMotionControlFramePeriod(Constants.kDriveMotionControlFramePeriod);
    	mRightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,  10, Constants.kCANTimeout);
    	mRightMaster.configVoltageCompSaturation(Constants.kDriveVoltageScale, Constants.kCANTimeout);
    	mRightMaster.enableVoltageCompensation(true);
    	mRightMaster.configOpenloopRamp(Constants.kDriveOpenLoopRampRate, Constants.kCANTimeout);
    	mRightMaster.configNeutralDeadband(Constants.kDriveDeadband, Constants.kCANTimeout);
    	mRightMaster.configContinuousCurrentLimit(Constants.kTalonDriveContinuousCurrentLimit, Constants.kCANTimeout);
    	mRightMaster.configPeakCurrentLimit(Constants.kTalonDrivePeakCurrentLimit, Constants.kCANTimeout);
    	mRightMaster.configPeakCurrentDuration(Constants.kTalonDrivePeakCurrentDuration, Constants.kCANTimeout);
    	mRightMaster.enableCurrentLimit(true);
    	mRightMaster.configClosedloopRamp(Constants.kDriveClosedLoopRampTime, Constants.kCANTimeout);
    	mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, Constants.kCANTimeout);
    	mRightMaster.configVelocityMeasurementWindow(64, Constants.kCANTimeout);
    	

    	mRightMaster.config_kP(Constants.kVelocitySlot, Constants.kVelocity_kP, Constants.kCANTimeout);
    	mRightMaster.config_kI(Constants.kVelocitySlot, Constants.kVelocity_kI, Constants.kCANTimeout);
    	mRightMaster.config_kD(Constants.kVelocitySlot, Constants.kVelocity_kD, Constants.kCANTimeout);
    	mRightMaster.config_kF(Constants.kVelocitySlot, Constants.kVelocity_kF, Constants.kCANTimeout);
    	
    	
    	
    	//vision ctrl gains
    	mRightMaster.config_kP(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kP, Constants.kCANTimeout); //slot, value, timeout
    	mRightMaster.config_kI(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kI, Constants.kCANTimeout);
    	mRightMaster.config_kD(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kD, Constants.kCANTimeout);
    	mRightMaster.config_kF(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kF, Constants.kCANTimeout);
    	mRightMaster.configMotionCruiseVelocity(Constants.kDrivetrainCruiseVelocity, Constants.kCANTimeout);
    	mRightMaster.configMotionAcceleration(Constants.kDrivetrainAcceleration, Constants.kCANTimeout);
    	
    	
    	mRightSlave1 = new WPI_TalonSRX(8);
    	mRightSlave1.follow(mRightMaster);
    	mRightSlave1.setInverted(true);
    	mRightSlave1.configOpenloopRamp(Constants.kDriveOpenLoopRampRate, Constants.kCANTimeout);
    	mRightSlave1.configNeutralDeadband(Constants.kDriveDeadband, Constants.kCANTimeout);

    	
    	mRightSlave2 = new WPI_TalonSRX(9);
    	mRightSlave2.follow(mRightMaster);
    	mRightSlave2.setInverted(true);
    	mRightSlave2.configOpenloopRamp(Constants.kDriveOpenLoopRampRate, Constants.kCANTimeout);
    	mRightSlave2.configNeutralDeadband(Constants.kDriveDeadband, Constants.kCANTimeout);
    	
    	rightHighGearSensor = new DigitalInput(3);
    	rightLowGearSensor = new DigitalInput(2);
    
    	mLeftMaster = new WPI_TalonSRX(0);
    	mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kCANTimeout);
    	mLeftMaster.setInverted(false);
    	mLeftMaster.setSensorPhase(true);
    	mLeftMaster.configMotionProfileTrajectoryPeriod(Constants.kDriveMotionControlTrajectoryPeriod, Constants.kCANTimeout);
    	mLeftMaster.changeMotionControlFramePeriod(Constants.kDriveMotionControlFramePeriod);
    	mLeftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kCANTimeout);
    	mLeftMaster.configVoltageCompSaturation(Constants.kDriveVoltageScale, Constants.kCANTimeout);
    	mLeftMaster.enableVoltageCompensation(true);
    	mLeftMaster.configOpenloopRamp(Constants.kDriveOpenLoopRampRate, Constants.kCANTimeout);
    	mLeftMaster.configNeutralDeadband(Constants.kDriveDeadband, Constants.kCANTimeout);
    	mLeftMaster.configContinuousCurrentLimit(Constants.kTalonDriveContinuousCurrentLimit, Constants.kCANTimeout);
    	mLeftMaster.configPeakCurrentLimit(Constants.kTalonDrivePeakCurrentLimit, Constants.kCANTimeout);
    	mLeftMaster.configPeakCurrentDuration(Constants.kTalonDrivePeakCurrentDuration, Constants.kCANTimeout);
    	mLeftMaster.enableCurrentLimit(true);
    	mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, Constants.kCANTimeout);
    	mLeftMaster.configVelocityMeasurementWindow(64, Constants.kCANTimeout);
    
    	mLeftMaster.config_kP(Constants.kVelocitySlot, Constants.kVelocity_kP, Constants.kCANTimeout);
    	mLeftMaster.config_kI(Constants.kVelocitySlot, Constants.kVelocity_kI, Constants.kCANTimeout);
    	mLeftMaster.config_kD(Constants.kVelocitySlot, Constants.kVelocity_kD, Constants.kCANTimeout);
    	mLeftMaster.config_kF(Constants.kVelocitySlot, Constants.kVelocity_kF, Constants.kCANTimeout);
    	
    	//vision ctrl gains
    	mLeftMaster.config_kP(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kP, Constants.kCANTimeout); //slot, value, timeout
    	mLeftMaster.config_kI(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kI, Constants.kCANTimeout);
    	mLeftMaster.config_kD(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kD, Constants.kCANTimeout);
    	mLeftMaster.config_kF(Constants.kVisionCtrlSlot, Constants.kVisionCtrl_kF, Constants.kCANTimeout);
    	mLeftMaster.configMotionCruiseVelocity(Constants.kDrivetrainCruiseVelocity, Constants.kCANTimeout);
    	mLeftMaster.configMotionAcceleration(Constants.kDrivetrainAcceleration, Constants.kCANTimeout);
    	
    	mLeftSlave1 = new WPI_TalonSRX(1);
    	mLeftSlave1.follow(mLeftMaster);
    	mLeftSlave1.setInverted(false);
    	mLeftSlave1.configOpenloopRamp(Constants.kDriveOpenLoopRampRate, Constants.kCANTimeout);
    	mLeftSlave1.configNeutralDeadband(Constants.kDriveDeadband, Constants.kCANTimeout);
    	
    	mLeftSlave2 = new WPI_TalonSRX(2);
    	mLeftSlave2.follow(mLeftMaster);
    	mLeftSlave2.setInverted(false);
    	mLeftSlave2.configOpenloopRamp(Constants.kDriveOpenLoopRampRate, Constants.kCANTimeout);
    	mLeftSlave2.configNeutralDeadband(Constants.kDriveDeadband, Constants.kCANTimeout);
    	
    	leftHighGearSensor = new DigitalInput(1);
    	leftLowGearSensor = new DigitalInput(0);
    	
    	pcm1 = new Compressor();
    	pcm1.setClosedLoopControl(true);
    	
    	resetSensors();
    	LiveWindow.disableTelemetry(pdp);
    	setNeutralMode(NeutralMode.Coast);
    	
    	mIsHighGear = false;
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
    
    public int getLeftPositionTicks() {
    	return leftDrivePositionTicks;
    }
    
    public int getRightPositionTicks() {
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
    
    public boolean isStopped() {
    	return Math.abs(mLeftMaster.getSelectedSensorVelocity(0)) < 50  && Math.abs(mRightMaster.getSelectedSensorVelocity(0)) < 50;
    }
    
    public boolean getLeftHighGear() {
    	return !leftHighGearSensor.get();
    }
    public boolean getLeftLowGear() {
    	return !leftLowGearSensor.get();
    }
    public boolean getRightHighGear() {
    	return !rightHighGearSensor.get();
    }
    public boolean getRightLowGear() {
    	return !rightLowGearSensor.get();
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
    		mLeftMaster.selectProfileSlot(Constants.kVelocitySlot, 0);
    		mRightMaster.selectProfileSlot(Constants.kVelocitySlot, 0);
    	}
    	else {
    		setOpenLoopRampRate(Constants.kDriveOpenLoopRampRate);
    	}
    	currentDriveMode = mode;
    	
    }
    
    public void setHighGear(boolean highGear) {
    	mIsHighGear = highGear;
    }
    
    public void setNeutralMode(NeutralMode mode) {
    	if(neutralMode != mode) {
    		neutralMode = mode;
    		mLeftMaster.setNeutralMode(mode);
    		mLeftSlave1.setNeutralMode(mode);
    		mLeftSlave2.setNeutralMode(mode);
    		mRightMaster.setNeutralMode(mode);
    		mRightSlave1.setNeutralMode(mode);
    		mRightSlave2.setNeutralMode(mode);
    	}
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
    	SmartDashboard.putString("Neutral Mode", String.valueOf(neutralMode));
    	SmartDashboard.putBoolean("High Gear Left", getLeftHighGear());
    	SmartDashboard.putBoolean("Low Gear Left", getLeftLowGear());
    	SmartDashboard.putBoolean("High Gear Right", getRightHighGear());
    	SmartDashboard.putBoolean("Low Gear Right", getRightLowGear());
    	SmartDashboard.putNumber("Raw Left Speed", rawLeftSpeed);
    	SmartDashboard.putNumber("Raw Right Speed", rawRightSpeed);
    	SmartDashboard.putNumber("Desired Angle", desiredAngle);
    	SmartDashboard.putNumber("Current angle", navx.getAngle());
    	SmartDashboard.putNumber("Gyro adjustment", gyroPIDOutput.getOutput());
    	SmartDashboard.putNumber("Desired Left Speed", desiredLeftSpeed);
    	SmartDashboard.putNumber("Desired Right Speed", desiredRightSpeed);
    	SmartDashboard.putNumber("Left Voltage", mLeftMaster.getMotorOutputVoltage());
    	SmartDashboard.putNumber("Right Voltage", mLeftMaster.getMotorOutputVoltage());
    }
    
    private void updateSpeedAndPosition() {
    	leftDrivePositionTicks = mLeftMaster.getSelectedSensorPosition(0);
    	rightDrivePositionTicks =  mRightMaster.getSelectedSensorPosition(0);
    	leftDrivePositionInches = (double) leftDrivePositionTicks / Constants.kDriveEncoderCodesPerRev  * Constants.kRevToInConvFactor;
    	rightDrivePositionInches =  (double) rightDrivePositionTicks / Constants.kDriveEncoderCodesPerRev * Constants.kRevToInConvFactor;
    	leftDriveSpeedTicks = mLeftMaster.getSelectedSensorVelocity(0);
    	rightDriveSpeedTicks =  mRightMaster.getSelectedSensorVelocity(0);
    	leftDriveSpeedRPM = leftDriveSpeedTicks * (600.0/ Constants.kDriveEncoderCodesPerRev);
    	rightDriveSpeedRPM =  rightDriveSpeedTicks * (600.0/ Constants.kDriveEncoderCodesPerRev);
    }
  
    public void resetDriveEncoders() {
		mLeftMaster.set(ControlMode.Position, 0);
		mRightMaster.set(ControlMode.Position, 0);
		mLeftMaster.set(ControlMode.Velocity, 0);
		mRightMaster.set(ControlMode.Velocity, 0);
		mLeftMaster.set(ControlMode.MotionProfile, 0);
		mRightMaster.set(ControlMode.MotionProfile, 0);
		mLeftMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeout);
		mRightMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeout);
	}
    
    public void resetSensors() {
    	navx.reset();
    	resetDriveEncoders();
    	mRightMaster.disable();
    	mLeftMaster.disable();
    }
    
    public boolean getHighGear() {
    	return leftHighGearSensor.get() && rightHighGearSensor.get();
    }
    public boolean getLowGear() {
    	return leftLowGearSensor.get() && rightLowGearSensor.get();
    }
    public boolean getNeutral() {
    	return !leftLowGearSensor.get() && !leftHighGearSensor.get() && !rightLowGearSensor.get() && !rightHighGearSensor.get();
    }
    
    public double getLeftSpeedInS() {
    	return ((double)mLeftMaster.getSelectedSensorVelocity(0) / Constants.kDriveEncoderCodesPerRev) * (Constants.kRevToInConvFactor * 10);
    }
    public double getRightSpeedInS() {
    	return ((double)mRightMaster.getSelectedSensorVelocity(0) / Constants.kDriveEncoderCodesPerRev) * (Constants.kRevToInConvFactor * 10);
    }
    
    public void setFeedForward(double leftF, double rightF) {
    	mLeftMaster.config_kF(0, leftF, Constants.kCANTimeout);
    	mRightMaster.config_kF(0, rightF, Constants.kCANTimeout);
	}
    
    public void setOpenLoopRampRate(double seconds) {
    	mLeftMaster.configOpenloopRamp(seconds, 0);
    	mRightMaster.configOpenloopRamp(seconds, 0);
    }
    
    public boolean gyroInPosition() {
    	return gyroControl.onTarget();
    }
}
