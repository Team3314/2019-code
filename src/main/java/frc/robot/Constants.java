
package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Constants {


//Conversions	
		public static int kDriveEncoderCodesPerRev = 8192;
		
		public static double kDegToTicksConvFactor = .038888888888888;
		public static double kRevToInConvFactor = 1;
		public static double kRevToInConvFactorLowGear = 1.28167; //XXX OLD 7.90274223082 High Gear;
		public static double kRevToInConvFactorHighGear = 2.945;
		public static double kFeetToEncoderCodes = (12.0 *kDriveEncoderCodesPerRev) / kRevToInConvFactor;
		public static double kDriveTicksToInches = kRevToInConvFactor / kDriveEncoderCodesPerRev;
		public static double kFPSToTicksPer100ms = (kFeetToEncoderCodes / 10);
		public static double kVoltageToNativeTalonUnits = 1023.0/12.0;
		public static int kCANTimeout =0;

//Drivetrain
		public static int kTalonDriveContinuousCurrentLimit = 40;
		public static int kTalonDrivePeakCurrentLimit = 40;
		public static int kTalonDrivePeakCurrentDuration = 0;
		public static int kNEODriveCurrentLimit = 80;
		public static double kDriveDeadband = .1;
		public static double kDriveOpenLoopRampRate = 0;
		public static double kDriveVoltageScale = 12.0;
		public static double kDriveClosedLoopRampTime = 0;
		public static double kMaxSpeedLowGear = 9 * 12 * 60 / kRevToInConvFactorLowGear; //fps * 12 * 60 / kRevtoin
		public static double kMaxSpeedHighGear = 20.5 * 12 * 60 / kRevToInConvFactorHighGear;//fps * 12 * 60 / kRevtoin
		public static MotorType kSparkMotorType = CANSparkMaxLowLevel.MotorType.kBrushless;
		public static double kDrivePIDPeriod = .02;
		

//Elevator
		public static int kElevatorCruiseVelocity = 0;
		public static int kElevatorAcceleration = 0;
		public static double kElevatorVoltageScale = 0;
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

//Cargo Intake
		public static final int kIntakeContinuousCurrentLimit = 0; //last year 15a
		public static final int kIntakePeakCurrentLimit = 0; //last year 15a
		public static final int kIntakeCurrentDuration = 0; //last year 100ms
//PID Constants
	//Gyrolock
		public static double kGyroLock_kP = .015;
		public static double kGyroLock_kI = 0;
		public static double kGyroLock_kD = 0;
		public static double kGyroLock_kF = 0;
	//Velocity
		public static double kVelocity_kP = 2e-6;
		public static double kVelocity_kI = 0;
		public static double kVelocity_kD = 0;
		public static double kVelocity_kF = 1.8e-4;
		public static double kVelocity_kIZone = 0;
		public static double kVelocity_MaxOutput = 1;
		public static int kVelocitySlot = 1;
	//Vision
		public static double kVisionCtrl_kP = .015;
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

	//Motion Profile	
		public static final double kMotionProfile_kP = 0;
		public static final double kMotionProfile_kI = 0;
		public static final double kMotionProfile_kD = 0;

	//Motion Profile Heading
		public static double kMotionProfileHeading_kP =  .025;
		public static double kMotionProfileHeading_kI = 0;
		public static double kMotionProfileHeading_kD =0;
		public static double kMotionProfileHeading_kF = 0;
		public static double kMotionProfileHeading_kA = 0;

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
		public static int kPulleyDiameter = 0;

	//Backward High
		public static final int kMotionProfileLeftBackHigh_kV = 0;
		public static final int kMotionProfileLeftBackHigh_kA = 0;
		public static final int kMotionProfileRightBackHigh_kV = 0;
		public static final int kMotionProfileRightBackHigh_kA = 0;
		public static final int kMotionProfileLeftBackHigh_Intercept = 0;
		public static final int kMotionProfileRightBackHigh_Intercept = 0;
	
	//Backward Low
		public static final int kMotionProfileLeftBackLow_kV = 0;
		public static final int kMotionProfileLeftBackLow_kA = 0;
		public static final int kMotionProfileRightBackLow_kV = 0;
		public static final int kMotionProfileRightBackLow_kA = 0;
		public static final int kMotionProfileLeftBackLow_Intercept = 0;
		public static final int kMotionProfileRightBackLow_Intercept = 0;

	//Forward High
		public static final int kMotionProfileLeftForeHigh_kV = 0;
		public static final int kMotionProfileLeftForeHigh_kA = 0;
		public static final int kMotionProfileRightForeHigh_kV = 0;
		public static final int kMotionProfileRightForeHigh_kA = 0;
		public static final int kMotionProfileLeftForeHigh_Intercept = 0;
		public static final int kMotionProfileRightForeHigh_Intercept = 0;

	//Forward Low
		public static final int kMotionProfileLeftForeLow_kV = 0;
		public static final int kMotionProfileLeftForeLow_kA = 0;
		public static final int kMotionProfileRightForeLow_kA = 0;
		public static final int kMotionProfileRightForeLow_kV = 0;
		public static final int kMotionProfileLeftForeLow_Intercept = 0;
		public static final int kMotionProfileRightForeLow_Intercept = 0;
		
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
	
	//Hatch mechanism
		public static Value kGripperUp = Value.kForward;
		public static Value kGripperDown = Value.kReverse;
		public static Value kSliderIn = Value.kForward;
		public static Value kSliderOut = Value.kReverse;

}