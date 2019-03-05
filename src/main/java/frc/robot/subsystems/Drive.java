package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.CustomPIDOutput;
import frc.robot.Robot;
import frc.robot.infrastructure.Drivetrain;
import frc.robot.infrastructure.EncoderAdapter;
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

    public enum DriveMode {
		IDLE,
		TANK,
        GYROLOCK,
        GYROLOCK_LEFT,
        GYROLOCK_RIGHT,
        POSITION,
        VISION_CONTROL, 
        MOTION_PROFILE
	}

    //Control Modes
    private DriveMode currentDriveMode = DriveMode.TANK;
    SpeedControllerMode controlMode = SpeedControllerMode.kIdle;
    
    //Hardware states
    private boolean mIsHighGear, elevatorUp, velocityControl = false, distanceSensorTriggered, lastDistanceSensorTriggered, atTarget = false, driveDistance;

    private IdleMode idleMode;
    private double rawLeftSpeed, rawRightSpeed, desiredAngle, cameraTurnAngle, tickToInConversion, speedCap, neoOffsetL, 
    neoOffsetR, maxDeccel, maxSpeed, cameraDistance, distanceSensorBookmark = 0, gyroDriveDistance;
    
    private double leftRioDrivePositionInches, rightRioDrivePositionInches, leftRioDrivePositionTicks, rightRioDrivePositionTicks, leftRioDriveSpeedTicks, rightRioDriveSpeedTicks, 
        leftRioDriveSpeedInches, rightRioDriveSpeedInches;

    private double leftNeoDrivePositionInches, rightNeoDrivePositionInches, 
    leftNeoDriveSpeedInches, rightNeoDriveSpeedInches, leftNeoInchesHighGear, leftNeoInchesLowGear, rightNeoInchesHighGear, rightNeoInchesLowGear, neoInchesPerRev;

    private double stoppingDistance, calculatedVelocity, targetDistance;

    private double[] gryoAngleHistory = new double[200];
    private int gyroAngleHistoryStoreIndex = 0;
    private int gyroAngleGetIndex;

    private PIDController gyroControl;
    private CustomPIDOutput gyroPIDOutput;

    private double visionOffset;

    private EncoderAdapter leftRioEncoder, rightRioEncoder;

    private AnalogInput distanceSensor;

    private Camera camera;
    public Drive(EncoderTransmission left, EncoderTransmission right, AHRS gyro, DoubleSolenoid shifter, EncoderAdapter leftEnc, EncoderAdapter rightEnc, AnalogInput distanceSensor){
        super(left, right);
        camera = Robot.camera;
        leftRioEncoder = leftEnc;
        rightRioEncoder = rightEnc;
        this.distanceSensor = distanceSensor;

    	
		//Hardware
    	this.shifter = shifter;
        navx = gyro;
        

        gyroPIDOutput = new CustomPIDOutput();
    	gyroControl = new PIDController(Constants.kGyroLock_kP, Constants.kGyroLock_kI, Constants.kGyroLock_kD,
            Constants.kGyroLock_kF, navx, gyroPIDOutput);
		//Sets the PID controller to treat 180 and -180 to be the same point, 
		//so that when turning the robot takes the shortest path instead of going the long way around
		//Effectively changes PID input from a line to a circle
		gyroControl.setOutputRange(-Constants.kGyroOutputRange, Constants.kGyroOutputRange);		// Limits speed of turn to prevent overshoot
        gyroControl.setAbsoluteTolerance(Constants.kAbsoluteGyroTolerance);
        gyroControl.setInputRange(-180, 180);
        gyroControl.setContinuous(true);

    }

    public void update(){
       if(mIsHighGear) {
            leftDrive.setEncoderDistancePerPulse(Constants.kNeoTicksToInHighGear);
            rightDrive.setEncoderDistancePerPulse(Constants.kNeoTicksToInHighGear);
            shifter.set(Constants.kHighGear);
            leftNeoInchesHighGear =  getLeftNeoPosition() - leftNeoInchesLowGear;
            rightNeoInchesHighGear = getRightNeoPosition() - rightNeoInchesLowGear;
            speedCap = Constants.kRaisedElevatorDriveSpeedCap / Constants.kMaxSpeedHighGear;
            neoInchesPerRev = Constants.kRevToInConvFactorHighGear;
            maxSpeed = Constants.kMaxSpeedHighGear;
            maxDeccel = Constants.kMaxDeccelerationHighGear;

    	}
    	else {
            leftDrive.setEncoderDistancePerPulse(Constants.kNeoTicksToInLowGear);
            rightDrive.setEncoderDistancePerPulse(Constants.kNeoTicksToInLowGear);
            shifter.set(Constants.kLowGear);
            leftNeoInchesLowGear =  getLeftNeoPosition() - leftNeoInchesHighGear;
            rightNeoInchesLowGear = getRightNeoPosition() - rightNeoInchesHighGear;
            speedCap = Constants.kRaisedElevatorDriveSpeedCap / Constants.kMaxSpeedLowGear;
            neoInchesPerRev = Constants.kRevToInConvFactorLowGear;
            maxSpeed = Constants.kMaxSpeedLowGear;
            maxDeccel = Constants.kMaxDeccelerationLowGear;

        }
        

        tickToInConversion = neoInchesPerRev / Constants.kNEODriveEncoderCodesPerRev;
        distanceSensorTriggered = getDistanceSensor();
        if(distanceSensorTriggered && !lastDistanceSensorTriggered) {
            distanceSensorBookmark = getAverageRioPosition();
        }
        if(velocityControl) {
            controlMode = SpeedControllerMode.kVelocity;
        }
        else {

            controlMode = SpeedControllerMode.kDutyCycle;
        }

        gryoAngleHistory[gyroAngleHistoryStoreIndex] = getAngle();
        gyroAngleHistoryStoreIndex++;
        gyroAngleHistoryStoreIndex %= 199;
        gyroAngleGetIndex = gyroAngleHistoryStoreIndex - 4;

        if(camera.isTargetInView()) {
            cameraTurnAngle = getDelayedGyroAngle() + camera.getTargetHorizError();
            cameraDistance = camera.getDistance();
            targetDistance = getAverageRioPosition() + cameraDistance;
        }
        if(gyroControl.getError() < 5) {
            gyroControl.setI(Constants.kGyroLock_kI);
        }  
        else {
            gyroControl.setI(0);
        }
        atTarget = (getAverageRioPosition() - distanceSensorBookmark) >= 10;
        updateSpeedAndPosition();
        switch(currentDriveMode) {
            case IDLE:
                setIdleMode(IdleMode.kBrake);
                rawLeftSpeed = 0;
                rawRightSpeed = 0;
                break;
            case TANK:
                rawLeftSpeed = leftDemand;
                rawRightSpeed = rightDemand;
                setIdleMode(IdleMode.kBrake);
                break;
            case GYROLOCK:
                if(driveDistance) {
                    calculatedVelocity = calcVelocity(0, gyroDriveDistance);
                    if((leftDemand * maxSpeed) > calculatedVelocity) {
                        leftDemand = calculatedVelocity / maxSpeed;
                        rightDemand = calculatedVelocity / maxSpeed;
                    }
                }
                rawLeftSpeed = leftDemand + gyroPIDOutput.getOutput();
                rawRightSpeed = rightDemand - gyroPIDOutput.getOutput();
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                break;
            case GYROLOCK_LEFT:
                rawLeftSpeed = leftDemand + gyroPIDOutput.getOutput();
                rawRightSpeed = 0;
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                break;
            case GYROLOCK_RIGHT:
                rawLeftSpeed = 0;
                rawRightSpeed = rightDemand - gyroPIDOutput.getOutput();
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                break;
            case VISION_CONTROL:
                if(!gyroInPosition()) {
                    //leftDemand = 0;
                    //rightDemand = 0;
                }
                calculatedVelocity = calcVelocity(Constants.kSafeImpactSpeed, getDistanceToTarget());
                if((leftDemand * maxSpeed) > calculatedVelocity) {
                    leftDemand = calculatedVelocity / maxSpeed;
                    rightDemand = calculatedVelocity / maxSpeed;
                }
                rawLeftSpeed = leftDemand + gyroPIDOutput.getOutput();
                rawRightSpeed = rightDemand - gyroPIDOutput.getOutput();
                setDesiredAngle(cameraTurnAngle + visionOffset);
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                break;
            case POSITION:
                rawLeftSpeed = leftDemand;
                rawRightSpeed = rightDemand;
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kPosition;
                break;
            case MOTION_PROFILE:
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kVelocity;
                break;
            }
            if(velocityControl) {
                    rawLeftSpeed *= Constants.kMaxSpeedRevs;
                    rawRightSpeed *= Constants.kMaxSpeedRevs;
            }
            if(elevatorUp) {
                setOpenLoopRampTime(Constants.kRaisedElevatorDriveRampRate);
                if(rawLeftSpeed > speedCap) {
                    rawLeftSpeed = speedCap;
                }
                if(rawRightSpeed > speedCap) {
                    rawRightSpeed = speedCap;
                }
                if(rawLeftSpeed > speedCap) {
                    rawLeftSpeed = speedCap;
                }
                if(rawRightSpeed > speedCap) {
                    rawRightSpeed = speedCap;
                }
            }
            else {
                setOpenLoopRampTime(Constants.kDriveRampRate);
                setClosedLoopRampTime(Constants.kDriveRampRate);
            }
        rightDrive.set(rawRightSpeed, controlMode);
        leftDrive.set(rawLeftSpeed, controlMode);
        lastDistanceSensorTriggered = distanceSensorTriggered;
    }

    public void setIdleMode(IdleMode mode) {
    	if(idleMode != mode) {
            idleMode = mode;

            leftDrive.setIdleMode(mode);
            rightDrive.setIdleMode(mode);
    	}
    }
    
    public void setDesiredAngle(double angle) {
        angle *= -1;
        if(angle < -180) {
            angle += 360;
        }
        else if(angle > 180){
            angle -= 360;
        }
    	desiredAngle = angle;
    }
    
    public double getDesiredAngle() {
    	return -desiredAngle;
    }
    
    public double getAngle() {
    	return -navx.getYaw();
    }

    public double getAcceleration(){
        return Math.round(100* navx.getWorldLinearAccelY());
    }

    public double getLeftNeoPositionTicks() {
        return leftDrive.getPosition() - neoOffsetL;
    }

    public double getRightNeoPositionTicks() {
        return rightDrive.getPosition() - neoOffsetR;
    }
    
    public double getLeftNeoPosition() {
    	return leftNeoDrivePositionInches;
    }
    
    public double getRightNeoPosition() {
    	return rightNeoDrivePositionInches;
    }
    
    public double getAverageNeoPosition() {
    	return (leftNeoDrivePositionInches-rightNeoDrivePositionInches)/2;
    }

    public double getLeftRioPositionTicks() {
        return -leftRioEncoder.getEncoderCounts();
    }

    public double getRightRioPositionTicks() {
        return rightRioEncoder.getEncoderCounts();
    }
    
    public double getLeftRioPosition() {
    	return leftRioDrivePositionInches;
    }
    
    public double getRightRioPosition() {
    	return rightRioDrivePositionInches;
    }
    
    public double getAverageRioPosition() {
    	return (leftRioDrivePositionInches + rightRioDrivePositionInches)/2;
    }

    public double getAverageRioSpeed() {
        return (leftRioDriveSpeedInches + rightRioDriveSpeedInches) / 2;
    }
    public double getAverageNeoSpeed() {
        return (leftNeoDriveSpeedInches + rightNeoDriveSpeedInches) / 2;
    }
    
    public double getStoppingDistance() {
        return stoppingDistance;
    }
    
    public void setDriveMode(DriveMode mode) {
        if(mode != currentDriveMode) {
            if(mode == DriveMode.GYROLOCK || mode == DriveMode.GYROLOCK_LEFT || mode == DriveMode.GYROLOCK_RIGHT) {
                gyroControl.enable();
                setGyroDriveDistance(0);
                setDesiredAngle(getAngle());
            }
            else if(mode == DriveMode.VISION_CONTROL) {
                gyroControl.enable();
            }
            else {
                gyroControl.disable();
            }

            if(mode == DriveMode.MOTION_PROFILE) {

            }
            currentDriveMode = mode;
        }
    }

    public void updateSpeedAndPosition() {
        leftNeoDrivePositionInches = leftNeoInchesHighGear + leftNeoInchesLowGear;
        rightNeoDrivePositionInches = rightNeoInchesHighGear + rightNeoInchesLowGear;
        leftNeoDriveSpeedInches = leftDrive.getVelocity();
        rightNeoDriveSpeedInches = -rightDrive.getVelocity();
        leftRioDrivePositionTicks = getLeftRioPositionTicks();
        rightRioDrivePositionTicks = getRightRioPositionTicks();
        leftRioDrivePositionInches = leftRioDrivePositionTicks * Constants.kDriveTicksToInches;
        rightRioDrivePositionInches = rightRioDrivePositionTicks * Constants.kDriveTicksToInches;
        leftRioDriveSpeedTicks = leftDrive.getVelocity();
        rightRioDriveSpeedTicks = -rightDrive.getVelocity();
        leftRioDriveSpeedInches = leftRioDriveSpeedTicks * tickToInConversion;
        rightRioDriveSpeedInches = rightRioDriveSpeedTicks * tickToInConversion;

    }
    
    public void setHighGear(boolean highGear) {
        mIsHighGear = highGear;
    }
    
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Left NEO Encoder Inches", getLeftNeoPosition());
    	SmartDashboard.putNumber("Right NEO Encoder Inches", getRightNeoPosition());
        SmartDashboard.putNumber("Left NEO Encoder Speed Inches", leftNeoDriveSpeedInches);
        SmartDashboard.putNumber("Right NEO Encoder Speed Inches", rightNeoDriveSpeedInches);
        SmartDashboard.putNumber("Avg. NEO Position", getAverageNeoPosition());
    	SmartDashboard.putNumber("Left Rio Encoder Position Ticks", leftRioDrivePositionTicks);
    	SmartDashboard.putNumber("Right Rio Encoder Position Ticks", rightRioDrivePositionTicks);
    	SmartDashboard.putNumber("Left Rio Encoder Inches", getLeftRioPosition());
    	SmartDashboard.putNumber("Right Rio Encoder Inches", getRightRioPosition());
    	SmartDashboard.putNumber("Left EO Encoder Speed RPS", leftRioDriveSpeedTicks);
        SmartDashboard.putNumber("Right Rio Encoder Speed RPS", rightRioDriveSpeedTicks);
        SmartDashboard.putNumber("Left Rio Encoder Speed Inches", leftRioDriveSpeedInches);
        SmartDashboard.putNumber("Right Rio Encoder Speed Inches", rightRioDriveSpeedInches);
        SmartDashboard.putNumber("Avg. Rio Position", getAverageRioPosition());
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
        SmartDashboard.putNumber("Left Velocity Difference", leftNeoDriveSpeedInches - (rawLeftSpeed * maxSpeed));
        SmartDashboard.putNumber("Right Velocity Difference", rightNeoDriveSpeedInches - (rawRightSpeed * maxSpeed));
    	SmartDashboard.putNumber("Desired Angle", getDesiredAngle());
    	SmartDashboard.putNumber("Current angle", getAngle());
        SmartDashboard.putNumber("Gyro adjustment", gyroPIDOutput.getOutput());
        SmartDashboard.putBoolean("Gyro Turn Done", gyroInPosition());
    	SmartDashboard.putNumber("Left Voltage", leftDrive.getOutputVoltage());
        SmartDashboard.putNumber("Right Voltage", rightDrive.getOutputVoltage());
        SmartDashboard.putNumber("Accelerometer", getAcceleration());
        SmartDashboard.putBoolean("Elevator Up", elevatorUp);
        SmartDashboard.putBoolean("high Gear", mIsHighGear);
        SmartDashboard.putNumber("Stopping Distance", stoppingDistance);
        SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
        SmartDashboard.putNumber("Calculated Velocity", calculatedVelocity);
        SmartDashboard.putBoolean("Distace Sensor", getDistanceSensor());
        SmartDashboard.putBoolean("At Target", getAtTarget());
        SmartDashboard.putNumber("Camera Distance", cameraDistance);
    }
  
    public void resetDriveEncoders() {
        neoOffsetL = leftDrive.getPosition();   
        neoOffsetR = rightDrive.getPosition();
        leftRioEncoder.zero();
        rightRioEncoder.zero();
        leftNeoInchesHighGear = 0;
        leftNeoInchesLowGear = 0;
        rightNeoInchesHighGear = 0;
        rightNeoInchesLowGear = 0;
    }
    
    public void resetSensors() {
    	navx.reset();
    	resetDriveEncoders();
    }
    
    public boolean gyroInPosition() {
    	return gyroControl.onTarget();
    }

    public boolean driveInPosition() {
        return false;
    }

    public void setClosedLoopRampTime(double time) {
        leftDrive.setClosedLoopRampTime(time);
        rightDrive.setClosedLoopRampTime(time);
    }

    public void setOpenLoopRampTime(double time){ 
        leftDrive.setOpenLoopRampTime(time);
        rightDrive.setOpenLoopRampTime(time);
    }

    public void setElevatorUp(boolean elevatorUp) {
        this.elevatorUp = elevatorUp;
    }
    public boolean collision(){
        return Math.round(100* navx.getWorldLinearAccelY()) > 55 ;
    }

    public void setVelocityControl(boolean velocityControl) {
        this.velocityControl = velocityControl;
    }
    public double getDistanceToTarget() {
        return targetDistance - getAverageRioPosition() - 12;
    } 
    public boolean getDistanceSensor() {
        return distanceSensor.getVoltage() <= Constants.kCargoSensorVoltageThreshold;
    }
    public boolean getAtTarget() {
        return atTarget && getDistanceSensor();
    }

    private double calcVelocity(double speed, double distance) {
        double calcVel = (Math.sqrt((speed * speed) - (2 * maxDeccel * (distance))));
        if(Double.isNaN(calcVel) || calcVel < speed) 
            calcVel = speed;
        
        return calcVel;
    }

    public void setGyroDriveDistance(double distance) {
        gyroDriveDistance = distance;
        if(distance ==  0) {
            driveDistance = false;
        } 
        else {
            driveDistance = true;
        }
    }   

    public boolean getGyroDriveDone() {
        return Math.abs(gyroDriveDistance - getAverageRioPosition()) < 5;
    }
    
    public double getDelayedGyroAngle() {
        int index = gyroAngleHistoryStoreIndex - 1;
        if(index < 0)
            index += 200;
        return gryoAngleHistory[index];
    }

    public void setVisionOffset(double offset) {
        visionOffset = offset;
    }
}
