package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
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

        DoubleSolenoid pivotPiston;

    //Hatch mechanism
        DoubleSolenoid gripperPiston;
        DoubleSolenoid sliderPiston;

        Compressor compressor;
    

    public RobotMap() {
        shifter = new DoubleSolenoid(0, 1);

        navx = new AHRS(SPI.Port.kMXP);

        leftDriveEncoder = new EncoderAdapter(new Encoder(0, 1, false, EncodingType.k4X));

        rightDriveEncoder = new EncoderAdapter(new Encoder(2, 3, false, EncodingType.k4X));

        mLeftMaster = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftMaster.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mLeftMaster.setRampRate(Constants.kDriveOpenLoopRampRate);
        mLeftMaster.setCANTimeout(Constants.kCANTimeout);
        mLeftMaster.getPIDController().setP(Constants.kVelocity_kP, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setI(Constants.kVelocity_kI, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setD(Constants.kVelocity_kD, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setIZone(Constants.kVelocity_kIZone, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setFF(Constants.kVelocity_kF, Constants.kVelocitySlot);
        mLeftMaster.getPIDController().setOutputRange(-Constants.kVelocity_MaxOutput, Constants.kVelocity_MaxOutput, Constants.kVelocitySlot);


        mLeftSlave1 = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave1.follow(mLeftMaster, true);
        mLeftSlave1.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mLeftSlave1.setRampRate(Constants.kDriveOpenLoopRampRate);
        mLeftSlave1.setCANTimeout(Constants.kCANTimeout);

        mLeftSlave2 = new CANSparkMax(3, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave2.follow(mLeftMaster, false);
        mLeftSlave2.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mLeftSlave2.setRampRate(Constants.kDriveOpenLoopRampRate);
        mLeftSlave2.setCANTimeout(Constants.kCANTimeout);

        mRightMaster = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMaster.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mRightMaster.setRampRate(Constants.kDriveOpenLoopRampRate);
        mRightMaster.setCANTimeout(Constants.kCANTimeout);
        mRightMaster.getPIDController().setP(Constants.kVelocity_kP, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setI(Constants.kVelocity_kI, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setD(Constants.kVelocity_kD, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setIZone(Constants.kVelocity_kIZone, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setFF(Constants.kVelocity_kF, Constants.kVelocitySlot);
        mRightMaster.getPIDController().setOutputRange(-Constants.kVelocity_MaxOutput, Constants.kVelocity_MaxOutput, Constants.kVelocitySlot);

        mRightSlave1 = new CANSparkMax(5, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave1.follow(mRightMaster, true);
        mRightSlave1.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mRightSlave1.setRampRate(Constants.kDriveOpenLoopRampRate);
        mRightSlave1.setCANTimeout(Constants.kCANTimeout);

        mRightSlave2 = new CANSparkMax(6, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave2.follow(mRightMaster, false);
        mRightSlave2.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mRightSlave2.setRampRate(Constants.kDriveOpenLoopRampRate);
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
        mElevatorSlave = new WPI_TalonSRX(8);

        mElevatorMasterWrapper = new TalonSRX(mElevatorMaster);
        mElevatorSlaveWrapper = new TalonSRX(mElevatorSlave);

        elevatorMotors = new SmartSpeedController[] {mElevatorMasterWrapper, mElevatorSlaveWrapper};

        elevatorTransmission = new EncoderTransmission(elevatorMotors, mElevatorMasterWrapper);

    //Cargo intake
        mIntakeMaster = new WPI_TalonSRX(9);
        mOuttakeMaster = new WPI_TalonSRX(10);

        mIntakeMasterWrapper = new TalonSRX(mIntakeMaster);
        mOuttakeMasterWrapper = new TalonSRX(mOuttakeMaster);

        intakeMotors = new SmartSpeedController[] {mIntakeMasterWrapper};
        outtakeMotors = new SmartSpeedController[] {mOuttakeMasterWrapper};

        intakeTransmission = new Transmission(intakeMotors);
        outtakeTransmission = new Transmission(outtakeMotors);

        pivotPiston = new DoubleSolenoid(6,7);

    //Hatch mechanism
        gripperPiston = new DoubleSolenoid(2, 3);
        sliderPiston = new DoubleSolenoid(4, 5);

    //Compressor
        compressor = new Compressor();
        compressor.setClosedLoopControl(true);
    }
}