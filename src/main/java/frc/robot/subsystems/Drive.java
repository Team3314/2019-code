package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    private boolean mIsHighGear, elevatorUp, velocityControl = true, driveDistance, placingCargoOnRocket;

    private IdleMode idleMode;
    private double rawLeftSpeed, rawRightSpeed, arbFFLeft = 0, arbFFRight = 0, desiredAngle, cameraTurnAngle, tickToInConversion, speedCap,
     maxDeccel, maxSpeed, cameraDistance, gyroDriveDistance;
    
    private double leftRioDrivePositionInches, rightRioDrivePositionInches, leftRioDrivePositionTicks, rightRioDrivePositionTicks, leftRioDriveSpeedTicks, rightRioDriveSpeedTicks, 
        leftRioDriveSpeedInches, rightRioDriveSpeedInches;

    private double leftNeoDrivePositionInches, rightNeoDrivePositionInches, 
    leftNeoDriveSpeedInches, rightNeoDriveSpeedInches, leftNeoInchesHighGear, leftNeoInchesLowGear, rightNeoInchesHighGear, rightNeoInchesLowGear, neoInchesPerRev;

    private double stoppingDistance, calculatedVelocity, targetDistance;

    private double[] gryoAngleHistory = new double[200];
    private int gyroAngleHistoryStoreIndex = 0;

    private PIDController gyroControl;
    private CustomPIDOutput gyroPIDOutput;


    private double visionOffset;

    private EncoderAdapter leftRioEncoder, rightRioEncoder;

    private AnalogInput rightRocketSensor, leftRocketSensor;

    private DigitalInput laserStationSensor;

    private boolean lastHighGear;

    private Camera camera;
    public Drive(EncoderTransmission left, EncoderTransmission right, AHRS gyro, DoubleSolenoid shifter, 
        EncoderAdapter leftEnc, EncoderAdapter rightEnc, AnalogInput rightRocketSensor, AnalogInput leftRocketSensor, DigitalInput laserStationSensor){
        super(left, right);
        camera = Robot.camera;
        leftRioEncoder = leftEnc;
        rightRioEncoder = rightEnc;
        this.rightRocketSensor = rightRocketSensor;
        this.leftRocketSensor = leftRocketSensor;
        this.laserStationSensor = laserStationSensor;
    	
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
            if(lastHighGear)
                 shifter.set(Value.kOff);
            else
                shifter.set(Constants.kHighGear);
            leftNeoInchesHighGear =  getLeftNeoPosition() - leftNeoInchesLowGear;
            rightNeoInchesHighGear = getRightNeoPosition() - rightNeoInchesLowGear;
            speedCap = Constants.kRaisedElevatorDriveSpeedCap / Constants.kMaxSpeedHighGear;
            neoInchesPerRev = Constants.kRevToInConvFactorHighGear;
            maxSpeed = Constants.kMaxSpeedHighGear;
            maxDeccel = Constants.kMaxDeccelerationHighGear;    
            if(leftDemand > 0) 
                arbFFLeft = Constants.kMotionProfileLeftForeHigh_Intercept;
            else if(leftDemand < 0)
                arbFFLeft = Constants.kMotionProfileLeftBackHigh_Intercept;
            else {
                arbFFLeft = 0;
            }
            if(rightDemand > 0) 
                arbFFRight = Constants.kMotionProfileRightForeHigh_Intercept;
            else if(rightDemand < 0)
                arbFFRight = Constants.kMotionProfileRightBackHigh_Intercept;
            else {
                arbFFRight = 0;
            }


    	}
    	else {
            leftDrive.setEncoderDistancePerPulse(Constants.kNeoTicksToInLowGear);
            rightDrive.setEncoderDistancePerPulse(Constants.kNeoTicksToInLowGear);
            if(!lastHighGear)
                shifter.set(Value.kOff);
            else
                shifter.set(Constants.kLowGear);
            leftNeoInchesLowGear =  getLeftNeoPosition() - leftNeoInchesHighGear;
            rightNeoInchesLowGear = getRightNeoPosition() - rightNeoInchesHighGear;
            speedCap = Constants.kRaisedElevatorDriveSpeedCap / Constants.kMaxSpeedLowGear;
            neoInchesPerRev = Constants.kRevToInConvFactorLowGear;
            maxSpeed = Constants.kMaxSpeedLowGear;
            maxDeccel = Constants.kMaxDeccelerationLowGear;
            if(leftDemand > 0) 
                arbFFLeft = Constants.kMotionProfileLeftForeLow_Intercept;
            else if(leftDemand < 0)
                arbFFLeft = Constants.kMotionProfileLeftBackLow_Intercept;
            else {
                arbFFLeft = 0;
            }
            if(rightDemand > 0) 
                arbFFRight = Constants.kMotionProfileRightForeLow_Intercept;
            else if(rightDemand < 0)
                arbFFRight = Constants.kMotionProfileRightBackLow_Intercept;
            else {
                arbFFRight = 0;
            }

        }

        tickToInConversion = neoInchesPerRev / Constants.kNEODriveEncoderCodesPerRev;
        if(velocityControl)
            controlMode = SpeedControllerMode.kVelocity;
        gryoAngleHistory[gyroAngleHistoryStoreIndex] = getAngle();
        gyroAngleHistoryStoreIndex++;
        gyroAngleHistoryStoreIndex %= 199;

        if(camera.isTargetInView()) {
            cameraTurnAngle = getDelayedGyroAngle() + camera.getTargetHorizError();
            if(placingCargoOnRocket)
                cameraDistance = camera.getHighDistance();
            else
                cameraDistance = camera.getDistance();
            targetDistance = getAverageRioPosition() + cameraDistance;
        }
        if(gyroControl.getError() < 5) {
            gyroControl.setI(Constants.kGyroLock_kI);
        }  
        else {
            gyroControl.setI(0);
        } 
        if(elevatorUp) {
            setOpenLoopRampTime(Constants.kRaisedElevatorDriveRampRate);
            setClosedLoopRampTime(Constants.kRaisedElevatorDriveRampRate);            
            if(Math.abs(leftDemand) > speedCap) {
                leftDemand = speedCap;
            }
            if(Math.abs(rightDemand) > speedCap) {
                rightDemand = speedCap;
            }
        }
        else {
            setOpenLoopRampTime(Constants.kDriveRampRate);
            setClosedLoopRampTime(Constants.kDriveRampRate);
        }
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
        rightDrive.set(rawRightSpeed, controlMode, arbFFRight);
        leftDrive.set(rawLeftSpeed, controlMode, arbFFLeft);
        lastHighGear = mIsHighGear;
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
    
    public double getLeftNeoPosition() {
    	return leftNeoDrivePositionInches;
    }
    
    public double getRightNeoPosition() {
    	return rightNeoDrivePositionInches;
    }

    public double getAverageNeoPosition() {
    	return (leftNeoDrivePositionInches-rightNeoDrivePositionInches)/2;
    }

    public double getLeftNeoVelocity() {
        return leftDrive.getVelocity();
    }

    public double getRightNeoVelocity() {
        return -rightDrive.getVelocity();
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
        //TODO PICK DASHBOARD CALLS
        SmartDashboard.putBoolean("Rocket Sensor", getAtRocket());
        SmartDashboard.putBoolean("Station Sensor", getStationSensor());
        SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
        SmartDashboard.putNumber("Left Rio Encoder Position", leftRioDrivePositionInches); 
        SmartDashboard.putNumber("Right Rio Encoder Position", rightRioDrivePositionInches);

    }
  
    public void resetDriveEncoders() {
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

    public boolean getRightRocketSensor() {
        return rightRocketSensor.getVoltage() <= Constants.kOpticalSensorVoltageThreshold;
    }

    public boolean getLeftRocketSensor() {
        return leftRocketSensor.getVoltage() <= Constants.kOpticalSensorVoltageThreshold;
    }

    public boolean getAtRocket() {
        if(Constants.kPracticeBot) {
            return getRightRocketSensor();
        }
        return getLeftRocketSensor() || getRightRocketSensor();
    }
    public boolean getStationSensor() {
        if(Constants.kPracticeBot) {
            return getRightRocketSensor();
        }
        return !laserStationSensor.get();
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
        int index = gyroAngleHistoryStoreIndex - Constants.kGyroDelay;
        if(index < 0)
            index += 200;
        return gryoAngleHistory[index];
    }

    public void setVisionOffset(double offset) {
        visionOffset = offset;
    }

    public void setPlacingOnRocket(boolean rocket) {
        placingCargoOnRocket = rocket;
    }

    @Override
    public void debug() {

    }

    
}
