
package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Constants {

//Conversions	
		public static int kDriveEncoderCodesPerRev = 8192;
		public static double kDegToTicksConvFactor = .038888888888888;
		public static double kRevToInConvFactor = 7.90274223082;
		public static double kFeetToEncoderCodes = (12.0 *kDriveEncoderCodesPerRev) / kRevToInConvFactor;
		public static double kFPSToTicksPer100ms = (kFeetToEncoderCodes / 10);
		public static double kVoltageToNativeTalonUnits = 1023.0/12.0;

		public static int kCANTimeout = 0;

//Drivetrain
		public static int kTalonDriveContinuousCurrentLimit = 40;
		public static int kTalonDrivePeakCurrentLimit = 40;
		public static int kTalonDrivePeakCurrentDuration = 0;
		public static int kNEODriveCurrentLimit = 50;
		public static double kDriveDeadband = .1;
		public static double kDriveOpenLoopRampRate = .2;
		public static double kDriveVoltageScale = 12.0;
		public static double kDriveClosedLoopRampTime = 0;
		public static double kMaxSpeed = 14; //fps

//PID Constants
	//Gyrolock
		public static double kGyroLock_kP = .025;
		public static double kGyroLock_kI = 0;
		public static double kGyroLock_kD = 0;
		public static double kGyroLock_kF = 0;

		public static double kVelocity_kP = .1;
		public static double kVelocity_kI = 0;
		public static double kVelocity_kD = 0;
		public static double kVelocity_kF = .043;
		public static double kVelocity_kIZone = 0;
		public static int kVelocitySlot = 1;

		public static double kVisionCtrl_kP = .01;
		public static double kVisionCtrl_kI = 0;
		public static double kVisionCtrl_kD = 0;
		public static double kVisionCtrl_kF = 0;
		public static double kVisionCtrl_kIZone = 0;
		public static int kVisionCtrlSlot = 3;

		
//Drive Motion Profile
		public static int kDriveMotionControlFramePeriod = 5; //ms
		public static int kDriveMotionControlTrajectoryPeriod = 10; //ms
		public static int kDrivetrainAcceleration = (int)(12 * Constants.kFPSToTicksPer100ms);
		public static int kDrivetrainCruiseVelocity = 0;

	
//camera
		public static int kLEDOn = 0;
		public static int kLEDOff = 1;
		public static int kLEDBlink = 2;
		public static int kVisionProcessorMode = 0;
		public static int kDriverCameraMode = 1;
		public static int kSnapshotOff = 0;
		public static int kSnapshotOn = 1;
		
		public static double kTrackingHeight = 11-4; //cube - camera
		public static double kMountingAngle = 10; //degrees
		
		public static double kMinMotorCmd = 0.175;//0-1
		public static double kMaxTrackingRPM = 0;
	
//Gyro
		public static double kGyroOutputRange = .5;
		public static double kAbsoluteGyroTolerance = 3;

//Pneumatics
    //Gears
		public static Value kHighGear = Value.kForward;
		public static Value kLowGear = Value.kReverse;

}