package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.infrastructure.EncoderAdapter;
import frc.robot.infrastructure.EncoderTransmission;
import frc.robot.infrastructure.SmartSpeedController;
import frc.robot.infrastructure.SparkMax;
import frc.robot.infrastructure.TalonSRX;
import frc.robot.infrastructure.Transmission;

public class RobotMap {
    // Drivetrain
        DoubleSolenoid shifter;

        AHRS navx;

        EncoderAdapter leftDriveEncoder;
        EncoderAdapter rightDriveEncoder;

        CANSparkMax mLeftMaster;
        CANSparkMax mLeftSlave1;
        CANSparkMax mLeftSlave2;

        SparkMax mLeftMasterWrapper;
        SparkMax mLeftSlave1Wrapper;
        SparkMax mLeftSlave2Wrapper;

        SparkMax mRightMasterWrapper;
        SparkMax mRightSlave1Wrapper;
        SparkMax mRightSlave2Wrapper;
        
        CANSparkMax mRightMaster;
        CANSparkMax mRightSlave1;
        CANSparkMax mRightSlave2;

        SmartSpeedController[] leftDriveMotors;
        SmartSpeedController[] rightDriveMotors;

        EncoderTransmission leftDrive;
        EncoderTransmission rightDrive;


    //Elevator
        WPI_TalonSRX mElevatorMaster;
        WPI_TalonSRX mElevatorSlave;

        TalonSRX mElevatorMasterWrapper;
        TalonSRX mElevatorSlaveWrapper;

        SmartSpeedController[] elevatorMotors;

        EncoderTransmission elevatorTransmission;

    //Intake
        WPI_TalonSRX mIntakeMaster;
        WPI_TalonSRX mOuttakeMaster;

        TalonSRX mIntakeMasterWrapper;
        TalonSRX mOuttakeMasterWrapper;
        
        SmartSpeedController[] intakeMotors;
        SmartSpeedController[] outtakeMotors;

        Transmission intakeTransmission;
        Transmission outtakeTransmission;

        DoubleSolenoid intakePiston, intakeToGroundPiston;

        Solenoid highPressure;

    //Hatch mechanism
        DoubleSolenoid gripperPiston;
        DoubleSolenoid sliderPiston;

    //Climber
        DoubleSolenoid climberPiston;
            

        Solenoid leftLightRing;
        Solenoid rightLightRing;

        AnalogInput distanceSensor;

        Compressor compressor;
    

    public RobotMap() {
        shifter = new DoubleSolenoid(0, 2, 3);

        navx = new AHRS(SPI.Port.kMXP);

        leftDriveEncoder = new EncoderAdapter(new Encoder(0, 1, false, EncodingType.k4X));
        rightDriveEncoder = new EncoderAdapter(new Encoder(2, 3, false, EncodingType.k4X));

        mLeftMaster = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftMaster.setSmartCurrentLimit(Constants.kNEODriveStallCurrentLimit, Constants.kNEODriveFreeCurrentLimit);
        mLeftMaster.setOpenLoopRampRate(Constants.kDriveRampRate);
        mLeftMaster.setCANTimeout(Constants.kCANTimeout);
        mLeftMaster.setParameter(ConfigParameter.kInputDeadband, Constants.kDriveDeadband);
        mLeftMaster.getPIDController().setP(Constants.kVelocity_kP, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setI(Constants.kVelocity_kI, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setD(Constants.kVelocity_kD, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setIZone(Constants.kVelocity_kIZone, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setFF(Constants.kVelocity_kF, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setOutputRange(-Constants.kVelocity_MaxOutput, Constants.kVelocity_MaxOutput, Constants.kVelocitySlot);


        mLeftSlave1 = new CANSparkMax(5, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave1.follow(mLeftMaster, true);
        mLeftSlave1.setParameter(ConfigParameter.kInputDeadband, Constants.kDriveDeadband);
        mLeftSlave1.setSmartCurrentLimit(Constants.kNEODriveStallCurrentLimit, Constants.kNEODriveFreeCurrentLimit);
        mLeftSlave1.setOpenLoopRampRate(Constants.kDriveRampRate);
        mLeftSlave1.setCANTimeout(Constants.kCANTimeout);

        mLeftSlave2 = new CANSparkMax(6, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave2.follow(mLeftMaster, false);
        mLeftSlave2.setParameter(ConfigParameter.kInputDeadband, Constants.kDriveDeadband);
        mLeftSlave2.setSmartCurrentLimit(Constants.kNEODriveStallCurrentLimit, Constants.kNEODriveFreeCurrentLimit);
        mLeftSlave2.setOpenLoopRampRate(Constants.kDriveRampRate);
        mLeftSlave2.setCANTimeout(Constants.kCANTimeout);

        mRightMaster = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMaster.setSmartCurrentLimit(Constants.kNEODriveStallCurrentLimit, Constants.kNEODriveFreeCurrentLimit);
        mRightMaster.setOpenLoopRampRate(Constants.kDriveRampRate);
        mRightMaster.setCANTimeout(Constants.kCANTimeout);
        mRightMaster.setParameter(ConfigParameter.kInputDeadband, Constants.kDriveDeadband);
        mRightMaster.getPIDController().setP(Constants.kVelocity_kP, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setI(Constants.kVelocity_kI, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setD(Constants.kVelocity_kD, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setIZone(Constants.kVelocity_kIZone, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setFF(Constants.kVelocity_kF, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setOutputRange(-Constants.kVelocity_MaxOutput, Constants.kVelocity_MaxOutput, Constants.kVelocitySlot);


        mRightSlave1 = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave1.follow(mRightMaster, true);
        mRightSlave1.setParameter(ConfigParameter.kInputDeadband, Constants.kDriveDeadband);
        mRightSlave1.setSmartCurrentLimit(Constants.kNEODriveStallCurrentLimit, Constants.kNEODriveFreeCurrentLimit);
        mRightSlave1.setOpenLoopRampRate(Constants.kDriveRampRate);
        mRightSlave1.setCANTimeout(Constants.kCANTimeout);

        mRightSlave2 = new CANSparkMax(3, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave2.follow(mRightMaster, false);
        mRightSlave2.setParameter(ConfigParameter.kInputDeadband, Constants.kDriveDeadband);
        mRightSlave2.setSmartCurrentLimit(Constants.kNEODriveStallCurrentLimit, Constants.kNEODriveFreeCurrentLimit);
        mRightSlave2.setOpenLoopRampRate(Constants.kDriveRampRate);
        mRightSlave2.setCANTimeout(Constants.kCANTimeout);

        mLeftMasterWrapper = new SparkMax(mLeftMaster);
        mLeftSlave1Wrapper = new SparkMax(mLeftSlave1);
        mLeftSlave2Wrapper = new SparkMax(mLeftSlave2);

        mRightMasterWrapper = new SparkMax(mRightMaster);
        mRightSlave1Wrapper = new SparkMax(mRightSlave1);
        mRightSlave2Wrapper = new SparkMax(mRightSlave2);

        leftDriveMotors = new SmartSpeedController[] {mLeftMasterWrapper, mLeftSlave1Wrapper, mLeftSlave2Wrapper};
        rightDriveMotors = new SmartSpeedController[] {mRightMasterWrapper, mRightSlave1Wrapper, mRightSlave2Wrapper};

        
        leftDrive = new EncoderTransmission(leftDriveMotors, mLeftMasterWrapper);//, Constants.kDrivePIDPeriod);
        leftDrive.setInverted(false);
        rightDrive = new EncoderTransmission(rightDriveMotors, mRightMasterWrapper);//, Constants.kDrivePIDPeriod);
        rightDrive.setInverted(true);

    //Elevator
        mElevatorMaster = new WPI_TalonSRX(7);
        mElevatorMaster.configContinuousCurrentLimit(Constants.kElevatorContinuousCurrentLimit, Constants.kCANTimeout);
        mElevatorMaster.configPeakCurrentLimit(Constants.kElevatorPeakCurrentLimit, Constants.kCANTimeout);
        mElevatorMaster.configPeakCurrentDuration(Constants.kElevatorPeakCurrentDuration, Constants.kCANTimeout);
        mElevatorMaster.enableCurrentLimit(true);
        mElevatorMaster.selectProfileSlot(Constants.kElevatorSlot, 0);
        mElevatorMaster.config_kP(Constants.kElevatorSlot, Constants.kElevator_kP, Constants.kCANTimeout);
        mElevatorMaster.config_kI(Constants.kElevatorSlot, Constants.kElevator_kI, Constants.kCANTimeout);
        mElevatorMaster.config_kD(Constants.kElevatorSlot, Constants.kElevator_kD, Constants.kCANTimeout);
        mElevatorMaster.config_kF(Constants.kElevatorSlot, Constants.kElevator_kF, Constants.kCANTimeout);
        mElevatorMaster.config_IntegralZone(Constants.kElevatorSlot, Constants.kElevator_kIZone, Constants.kCANTimeout);
        mElevatorMaster.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kCANTimeout);
        mElevatorMaster.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kCANTimeout);
        mElevatorMaster.configMotionSCurveStrength(Constants.kElevatorSCurveStrength, Constants.kCANTimeout);
        mElevatorMaster.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, 8, Constants.kCANTimeout);
        mElevatorMaster.configClearPositionOnLimitR(false, Constants.kCANTimeout);
        mElevatorMaster.configNeutralDeadband(Constants.kElevatorDeadband, Constants.kCANTimeout);

        mElevatorMaster.configClosedloopRamp(Constants.kElevatorRampTime, Constants.kCANTimeout);
        mElevatorMaster.configOpenloopRamp(Constants.kElevatorRampTime, Constants.kCANTimeout);
        mElevatorMaster.configForwardSoftLimitThreshold(Constants.kMaxElevatorPosition);
        mElevatorMaster.configForwardSoftLimitEnable(false, Constants.kCANTimeout);
        mElevatorMaster.configReverseSoftLimitThreshold(Constants.kMinElevatorPosition);
        mElevatorMaster.configReverseSoftLimitEnable(false, Constants.kCANTimeout);

        mElevatorMaster.configVoltageCompSaturation(Constants.kElevatorVoltageScale, Constants.kCANTimeout);
        mElevatorMaster.enableVoltageCompensation(true);
        mElevatorMaster.setSensorPhase(false);
        mElevatorMaster.setInverted(true);

        mElevatorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, Constants.kCANTimeout);
        mElevatorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.kCANTimeout);
        
        mElevatorSlave = new WPI_TalonSRX(8);
        mElevatorSlave.follow(mElevatorMaster);
        mElevatorSlave.configNeutralDeadband(Constants.kElevatorDeadband, Constants.kCANTimeout);
        mElevatorSlave.setInverted(InvertType.OpposeMaster);


        mElevatorMasterWrapper = new TalonSRX(mElevatorMaster);
        mElevatorMasterWrapper.setDistancePerPulse(Constants.kElevatorInchesPerTick);
        mElevatorSlaveWrapper = new TalonSRX(mElevatorSlave);

        elevatorMotors = new SmartSpeedController[] {mElevatorMasterWrapper, mElevatorSlaveWrapper};

        elevatorTransmission = new EncoderTransmission(elevatorMotors, mElevatorMasterWrapper);

    //Cargo intake
        mIntakeMaster = new WPI_TalonSRX(9);
        mIntakeMaster.setInverted(true);
        mOuttakeMaster = new WPI_TalonSRX(10);
        mOuttakeMaster.setInverted(true);

        mIntakeMasterWrapper = new TalonSRX(mIntakeMaster);
        mOuttakeMasterWrapper = new TalonSRX(mOuttakeMaster);

        intakeMotors = new SmartSpeedController[] {mIntakeMasterWrapper};
        outtakeMotors = new SmartSpeedController[] {mOuttakeMasterWrapper};

        intakeTransmission = new Transmission(intakeMotors);
        outtakeTransmission = new Transmission(outtakeMotors);

        intakePiston = new DoubleSolenoid(0, 0, 1);
        intakeToGroundPiston = new DoubleSolenoid(2, 6, 7);
        highPressure = new Solenoid(2, 2);
        leftLightRing = new Solenoid(2, 0);
        rightLightRing = new Solenoid(2, 1);

    //Hatch mechanism
        gripperPiston = new DoubleSolenoid(1, 0, 1);
        sliderPiston = new DoubleSolenoid(1, 2, 3);

    //Compressor
        compressor = new Compressor();
        compressor.setClosedLoopControl(true);

        climberPiston = new DoubleSolenoid(0, 4, 5);

        distanceSensor = new AnalogInput(2);
    }
}