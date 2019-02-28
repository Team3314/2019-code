package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Constants {

	public static boolean practiceBot = false;


//Conversions	
		public static int kDriveEncoderCodesPerRev = 2048;// AMT-102v = 2048;
		public static int kNEODriveEncoderCodesPerRev = 42;
		
		public static double kDegToTicksConvFactor = .038888888888888;
		public static double kRioEncoderRevToInConvFactor = 11.211678832116788321167883211679;
		public static double kRevToInConvFactorLowGear = 1.2245; //first stage (12/44) * second stage(14/60) * wheel circumference (6.125 * pi)
		public static double kRevToInConvFactorHighGear = 3.5781; //first stage (12/44) * second stage(30/44) * wheel circumference (6.125 * pi)
		public static double kNeoTicksToInHighGear = kRevToInConvFactorHighGear / kNEODriveEncoderCodesPerRev;
		public static double kNeoTicksToInLowGear = kRevToInConvFactorLowGear / kNEODriveEncoderCodesPerRev;
		public static double kFeetToEncoderCodes = (12.0 *kDriveEncoderCodesPerRev) / kRioEncoderRevToInConvFactor;
		public static double kDriveTicksToInches = kRioEncoderRevToInConvFactor / kDriveEncoderCodesPerRev;
		public static double kFPSToTicksPer100ms = (kFeetToEncoderCodes / 10);
		public static double kVoltageToNativeTalonUnits = 1023.0/12.0;

		public static double kElevatorInchesPerTick = 4.4468368947179040128423054179364e-4;//0.97759262293478401818325242307913

		public static int kCANTimeout =0;
//Drivetrain
		public static int kTalonDriveContinuousCurrentLimit = 40;
		public static int kTalonDrivePeakCurrentLimit = 40;
		public static int kTalonDrivePeakCurrentDuration = 0;
		public static int kNEODriveStallCurrentLimit = 80;
		public static int kNEODriveFreeCurrentLimit = 20;
		public static double kMaxDeccelerationLowGear = 60; //inches/sec^2
		public static double kMaxDeccelerationHighGear = 60; //inches/sec^2
		public static double kSafeImpactSpeed = 36; //inches/sec
		public static double kDriveDeadband = .1; 
		public static double kDriveOpenLoopRampRate = .5;
		public static double kDriveVoltageScale = 12.0;
		public static double kDriveClosedLoopRampTime = 0;
		public static double kMaxSpeedLowGear = 9;// * 12 * 60 / kRevToInConvFactorLowGear; //fps * 12 * 60 / kRevtoin
		public static double kMaxSpeedHighGear = 20.5;// * 12 * 60 / kRevToInConvFactorHighGear;//fps * 12 * 60 / kRevtoin
		public static double kMaxSpeed = 5500;
		public static MotorType kSparkMotorType = CANSparkMaxLowLevel.MotorType.kBrushless;
		public static double kDrivePIDPeriod = .02;
		public static double kRaisedElevatorDriveRampRate = .5;
		public static double kRaisedElevatorDriveSpeedCap = kMaxSpeedLowGear;
		

//Elevator
		public static int kElevatorCruiseVelocity = (int)(62/ kElevatorInchesPerTick/ 10); //inches/sec -> ticks/100ms
		public static int kElevatorAcceleration = (int)(120 / kElevatorInchesPerTick / 10); //inches/sec/sec -> tick/100ms^2
		public static final int kElevatorSCurveStrength = 0;
		public static double kElevatorVoltageScale = 12.0;
		public static final double kElevatorRampTime = .1; 
		public static final int kElevatorContinuousCurrentLimit = 20;
		public static final int kElevatorPeakCurrentLimit = 35;
		public static final int kElevatorPeakCurrentDuration = 200;
		public static final double kElevatorDeadband = 0;
		public static final int kMaxElevatorPosition = 10000000;
		public static final int kMinElevatorPosition = (int)(-1 / kElevatorInchesPerTick);
		public static final int k775ProFreeSpeedRPM = 18730;
		public static final double kElevatorGearboxReduction = 1/13;
		public static final int kElevatorTicksPerRev = 8192;
		public static final double kMaxElevatorVelocity = 14000; //ticks/100ms

		public static final int kElevatorTolerance = (int)(1 / kElevatorInchesPerTick);

		public static final int kElevatorLowAccelerationThreshold = ((int)(30 / kElevatorInchesPerTick));
	
		public static final int kElevatorHatchPickup = (int)(5 / kElevatorInchesPerTick);
		public static final int kElevatorRaisedHatchPickup = (int)(7.5 / kElevatorInchesPerTick);
		public static final int kElevatorBallLevel1 = (int)(2.5 / kElevatorInchesPerTick);
		public static final int kElevatorBallLevel2 = (int)(27 / kElevatorInchesPerTick);
		public static final int kElevatorBallLevel3 = (int)(55 / kElevatorInchesPerTick);
		public static final int kElevatorHatchLevel1 = (int)(5 / kElevatorInchesPerTick);
		public static final int kElevatorHatchLevel2 = (int)(32 / kElevatorInchesPerTick);
		public static final int kElevatorHatchLevel3 = (int)(62 / kElevatorInchesPerTick);

		public static double kElevator_kP = .1;
		public static double kElevator_kI = 0;
		public static double kElevator_kD = 5;
		public static double kElevator_kF = 1023 / kMaxElevatorVelocity;//11.694760041671591487403907388346; // 1/max velocity * 1023 ( converts to talon native units)
		public static final int kElevator_kIZone = 0;
		public static int kElevatorSlot = 0;

//Cargo Intake
		public static final int kIntakeContinuousCurrentLimit = 0; //last year 15a
		public static final int kIntakePeakCurrentLimit = 0; //last year 15a
		public static final int kIntakeCurrentDuration = 0; //last year 100ms
//PID Constants
	//Gyrolock
		public static double kGyroLock_kP = .005;
		public static double kGyroLock_kI = 0;
		public static double kGyroLock_kD = 0;
		public static double kGyroLock_kF = 0;
	//Velocity
		public static double kVelocity_kP = 2e-6;
		public static double kVelocity_kI = 0;
		public static double kVelocity_kD = 0;
		public static double kVelocity_kF = 1.7e-4;
		public static double kVelocity_kIZone = 0;
		public static double kVelocity_MaxOutput = 1;
		public static int kVelocitySlot = 1;

	//Velocity High Gear
		public static double kHighGearVelocity_kP = 2e-6;
		public static double kHighGearVelocity_kI = 0;
		public static double kHighGearVelocity_kD = 0;
		public static double kHighGearVelocity_kF = 1.7e-4;
		public static double kHighGearVelocity_kIZone = 0;
		public static double kHighGearVelocity_MaxOutput = 1;
		public static int kHighGearVelocitySlot = 1;
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
		
//Drive Motion Profile
		public static int kDriveMotionControlFramePeriod = 5; //ms
		public static int kDriveMotionControlTrajectoryPeriod = 10; //ms
		public static int kDrivetrainAcceleration = (int)(12 * Constants.kFPSToTicksPer100ms);
		public static int kDrivetrainCruiseVelocity = 0;
		public static double kWheelDiameter = 6.125;
		public static boolean kNEOEncoders = false;



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
		public static double kGyroOutputRange = 1;
		public static double kAbsoluteGyroTolerance = 3;

//Pneumatics
    //Gears
		public static Value kHighGear = Value.kReverse;
		public static Value kLowGear = Value.kForward;
	
	//Hatch mechanism
		public static Value kGripperUp = Value.kReverse;
		public static Value kGripperDown = Value.kForward;
		public static Value kSliderIn = Value.kForward;
		public static Value kSliderOut = Value.kReverse;

	//Cargo Intake	
		public static final Value kIntakeDown = Value.kReverse;
		public static final Value kIntakeUp = Value.kForward; 
		public static final Value kIntakeClimberUp = Value.kForward;
		public static final Value kIntakeClimberDown = Value.kReverse;
		public static final double kCargoSensorVoltageThreshold = 3;


		public static final double kRightStopUpAngle = 82;


		public static final double kLeftStopUpAngle = 125;


		public static final double kRightStopDownAngle = 115;


		public static final double kLeftStopDownAngle = 85;


		public static final Value kClimberDown = Value.kForward;


		public static final Value kClimberUp = Value.kReverse;


}