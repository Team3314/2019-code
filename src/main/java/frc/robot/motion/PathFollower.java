package frc.robot.motion;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.Path;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveMode;

import edu.wpi.first.wpilibj.Notifier;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;

public class PathFollower {

	/**
	 * Class to follow paths generated by pathfinder
	 * Takes left and right paths (in feet), then calculates a velocity.
	 */
	
	private Drive drive = Robot.drive;
	private EncoderFollower left, right;
	private double leftVIntercept, rightVIntercept, lastHeading = 0, lastHeadingChange = 0, lastHeadingError = 0;
	private int direction;
	private int encoderCodesPerRev, leftOffset, rightOffset;
	private boolean pathFinished = false;
	
	class PeriodicRunnable implements java.lang.Runnable {
		@Override
		public void run() {
		    double leftSpeed = left.calculate((int)drive.getLeftRioPositionTicks() * direction) * direction + leftVIntercept; 
			double rightSpeed = right.calculate((int)drive.getRightRioPositionTicks() * direction) * direction + rightVIntercept;
			double gyroHeading = -drive.getAngle();
			double desiredHeading = Pathfinder.r2d(left.getHeading());
			double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
			double headingErrorChange = Pathfinder.boundHalfDegrees(angleDifference -  lastHeadingError);
			double angleSetpointChange = Pathfinder.boundHalfDegrees(desiredHeading - lastHeading);
			double angleAcceleration = angleSetpointChange - lastHeadingChange;
			double headingF = angleSetpointChange * Constants.kMotionProfileHeading_kF;
			double headingA = angleAcceleration * Constants.kMotionProfileHeading_kA;
			double turn = Constants.kMotionProfileHeading_kP  * angleDifference;
			double headingD = Constants.kMotionProfileHeading_kD * (headingErrorChange/.01) ;
			leftSpeed = leftSpeed - turn - headingF - headingA - headingD;
			rightSpeed = rightSpeed + turn + headingF + headingA + headingD;
			drive.set(leftSpeed, rightSpeed);
			lastHeading = desiredHeading;
			lastHeadingChange = angleSetpointChange;
			lastHeadingError = angleDifference;
			pathFinished = left.isFinished() && right.isFinished();
			if(pathFinished) {
				left.reset();
				right.reset();
				drive.setDriveMode(DriveMode.IDLE);
				notifier.stop();
			}
		}
	}
	
	private Notifier notifier = new Notifier(new PeriodicRunnable());
	
	//kV, kA, and Intercept are divided by 12 to convert them to duty cycle from voltage
	public void followPath(Path path) {
		drive.resetDriveEncoders();
		switch(path.getMode()) {
		case BACKWARD_HIGH:
			left = new EncoderFollower(Pathfinder.readFromCSV(path.getRightPath()));
			right = new EncoderFollower(Pathfinder.readFromCSV(path.getLeftPath()));
			left.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileLeftBackHigh_kV / 12, Constants.kMotionProfileLeftBackHigh_kA / 12);
			right.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileRightBackHigh_kV / 12, Constants.kMotionProfileRightBackHigh_kA / 12);
			leftVIntercept = Constants.kMotionProfileLeftBackHigh_Intercept / 12;
			rightVIntercept = Constants.kMotionProfileRightBackHigh_Intercept / 12;
			direction = -1;
			break;
		case BACKWARD_LOW:
			left = new EncoderFollower(Pathfinder.readFromCSV(path.getRightPath()));
			right = new EncoderFollower(Pathfinder.readFromCSV(path.getLeftPath()));
			left.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileLeftBackLow_kV / 12, Constants.kMotionProfileLeftBackLow_kA / 12);
			right.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileRightBackLow_kV / 12, Constants.kMotionProfileRightBackLow_kA / 12);
			leftVIntercept = Constants.kMotionProfileLeftBackLow_Intercept / 12;
			rightVIntercept = Constants.kMotionProfileRightBackLow_Intercept / 12;
			direction = -1;
			break;
		case FORWARD_HIGH:
			left = new EncoderFollower(Pathfinder.readFromCSV(path.getLeftPath()));
			right = new EncoderFollower(Pathfinder.readFromCSV(path.getRightPath()));
			left.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileLeftForeHigh_kV / 12, Constants.kMotionProfileLeftForeHigh_kA / 12);
			right.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileRightForeHigh_kV / 12, Constants.kMotionProfileRightForeHigh_kA / 12);
			leftVIntercept = Constants.kMotionProfileLeftForeHigh_Intercept / 12;
			rightVIntercept = Constants.kMotionProfileRightForeHigh_Intercept / 12;
			direction = 1;
			break;
		case FORWARD_LOW:
			left = new EncoderFollower(Pathfinder.readFromCSV(path.getLeftPath()));
			right = new EncoderFollower(Pathfinder.readFromCSV(path.getRightPath()));
			left.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileLeftForeLow_kV / 12, Constants.kMotionProfileLeftForeLow_kA / 12);
			right.configurePIDVA(Constants.kMotionProfile_kP, Constants.kMotionProfile_kI, Constants.kMotionProfile_kD, Constants.kMotionProfileRightForeLow_kV / 12, Constants.kMotionProfileRightForeLow_kA / 12);
			leftVIntercept = Constants.kMotionProfileLeftForeLow_Intercept / 12;
			rightVIntercept = Constants.kMotionProfileRightForeLow_Intercept / 12;
			direction = 1;
			break;
		}
		if(Constants.kNEOEncoders) {
			encoderCodesPerRev = Constants.kNEODriveEncoderCodesPerRev;
		}
		else {
			encoderCodesPerRev = Constants.kDriveEncoderCodesPerRev;
		}
		left.configureEncoder((int)drive.getLeftRioPositionTicks(), encoderCodesPerRev, Constants.kWheelDiameter / 12);
		right.configureEncoder((int)drive.getRightRioPositionTicks(), encoderCodesPerRev, Constants.kWheelDiameter / 12);
		pathFinished = false;
		
		drive.set(0, 0);
		drive.setDriveMode(DriveMode.MOTION_PROFILE);
		notifier.startPeriodic(0.01);
	}
	
	public boolean isDone() { 
		return pathFinished;
	}
	
	public void stop() {
		left = null;
		right = null;
		pathFinished = true;
		drive.setDriveMode(DriveMode.IDLE);
	}

}