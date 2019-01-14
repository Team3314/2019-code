
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

//Elevator
		public static int kElevatorCruiseVelocity = 0;
		public static int kElevatorAcceleration = 0;
		public static final double kElevatorOpenLoopRampRate = 0;
		public static final int kElevatorContinuousCurrentLimit = 0;
		public static final double kElevatorDeadband = 0;
		public static final int kElevatorPeakCurrentLimit = 0;
		public static final int kElevatorCurrentDuration = 0;
		public static final double kElevatorClosedLoopRampTime = 0;
		public static final int kMaxElevatorPosition = 0;
		public static final int kMinElevatorPosition = 0;

		public static final int kElevatorPickup = 0;
		public static final int kElevatorLevel1 = 0;
		public static final int kElevatorLevel2 = 0;
		public static final int kElevatorLevel3 = 0;

//PID Constants
	//Gyrolock
		public static double kGyroLock_kP = .025;
		public static double kGyroLock_kI = 0;
		public static double kGyroLock_kD = 0;
		public static double kGyroLock_kF = 0;
	//Velocity
		public static double kVelocity_kP = .1;
		public static double kVelocity_kI = 0;
		public static double kVelocity_kD = 0;
		public static double kVelocity_kF = .043;
		public static double kVelocity_kIZone = 0;
		public static int kVelocitySlot = 1;
	//Vision
		public static double kVisionCtrl_kP = .01;
		public static double kVisionCtrl_kI = 0;
		public static double kVisionCtrl_kD = 0;
		public static double kVisionCtrl_kF = 0;
		public static double kVisionCtrl_kIZone = 0;
		public static int kVisionCtrlSlot = 3;
	//Rio Position
		public static final double kRioPosition_kP = 0;
		public static final double kRioPosition_kI = 0;
		public static final double kRioPosition_kD = 0;
		public static final double kRioPosition_kF = 0;

	//Elevator
		public static double kElevator_kP = 0;
		public static double kElevator_kI = 0;
		public static double kElevator_kD = 0;
		public static double kElevator_kF = 0;
		public static int kElevatorSlot = 0;
		
//Drive Motion Profile
		public static int kDriveMotionControlFramePeriod = 5; //ms
		public static int kDriveMotionControlTrajectoryPeriod = 10; //ms
		public static int kDrivetrainAcceleration = (int)(12 * Constants.kFPSToTicksPer100ms);
		public static int kDrivetrainCruiseVelocity = 0;

	
//camera
		public static int kLEDDefault = 0;
		public static int kLEDOff = 1;
		public static int kLEDOn = 3;

		public static int kVisionProcessorMode = 0;
		public static int kDriverCameraMode = 1;
		
		public static int kSnapshotOff = 0;
		public static int kSnapshotOn = 1;
		
		public static double kCameraHeight = 13/16; //inches
		public static double kMountingAngle = 10; //degrees
	
//Gyro
		public static double kGyroOutputRange = .5;
		public static double kAbsoluteGyroTolerance = 3;

//Pneumatics
    //Gears
		public static Value kHighGear = Value.kForward;
		public static Value kLowGear = Value.kReverse;
		public static double kElevatorVoltageScale;

}