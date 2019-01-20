package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.infrastructure.SensorTransmission;
import frc.robot.infrastructure.SmartSpeedController;
import frc.robot.infrastructure.SparkMax;
import frc.robot.infrastructure.TalonSRX;

public class RobotMap {
    // Drivetrain
        AHRS navx;

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

        SensorTransmission leftDrive;
        SensorTransmission rightDrive;


    //Elevator
        WPI_TalonSRX mElevatorMaster;

        TalonSRX mElevatorMasterWrapper;

        SmartSpeedController[] elevatorMotors;

        SensorTransmission elevatorTransmission;

    //Intake
        WPI_TalonSRX mIntakeMaster;

        TalonSRX mIntakeMasterWrapper;
        
        SmartSpeedController[] intakeMotors;

        SensorTransmission intakeTransmission;

    //Hatch mechanism
        DoubleSolenoid gripperPiston;
        DoubleSolenoid sliderPiston;

    public RobotMap() {

        navx = new AHRS(SPI.Port.kMXP);

        mLeftMaster = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftMaster.setInverted(false);
        mLeftMaster.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mLeftMaster.setRampRate(Constants.kDriveOpenLoopRampRate);

        mLeftSlave1 = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave1.follow(mLeftMaster);
        mLeftSlave1.setInverted(true);
        mLeftSlave1.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mLeftSlave1.setRampRate(Constants.kDriveOpenLoopRampRate);

        mLeftSlave2 = new CANSparkMax(3, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave2.follow(mLeftMaster);
        mLeftSlave2.setInverted(false);
        mLeftSlave2.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mLeftSlave2.setRampRate(Constants.kDriveOpenLoopRampRate);

        mRightMaster = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMaster.setInverted(false);
        mRightMaster.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mRightMaster.setRampRate(Constants.kDriveOpenLoopRampRate);

        mRightSlave1 = new CANSparkMax(5, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave1.follow(mRightMaster);
        mRightSlave1.setInverted(true);
        mRightSlave1.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mRightSlave1.setRampRate(Constants.kDriveOpenLoopRampRate);

        mRightSlave2 = new CANSparkMax(6, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave2.follow(mRightMaster);
        mRightSlave2.setInverted(false);
        mRightSlave2.setSmartCurrentLimit(Constants.kNEODriveCurrentLimit);
        mRightSlave2.setRampRate(Constants.kDriveOpenLoopRampRate);

        mLeftMasterWrapper = new SparkMax(mLeftMaster);
        mLeftSlave1Wrapper = new SparkMax(mLeftSlave1);
        mLeftSlave2Wrapper = new SparkMax(mLeftSlave2);

        mRightMasterWrapper = new SparkMax(mRightMaster);
        mRightSlave1Wrapper = new SparkMax(mRightSlave1);
        mRightSlave2Wrapper = new SparkMax(mRightSlave2);

        leftDriveMotors = new SmartSpeedController[] {mLeftMasterWrapper, mLeftSlave1Wrapper, mLeftSlave2Wrapper};
        rightDriveMotors = new SmartSpeedController[] {mRightMasterWrapper, mRightSlave1Wrapper, mRightSlave2Wrapper};

        leftDrive = new SensorTransmission(leftDriveMotors, mLeftMasterWrapper);
        leftDrive.setInverted(true);
        rightDrive = new SensorTransmission(rightDriveMotors, mRightMasterWrapper);
        rightDrive.setInverted(false);


    //Elevator
        mElevatorMaster = new WPI_TalonSRX(7);

        mElevatorMasterWrapper = new TalonSRX(mElevatorMaster);

        elevatorMotors = new SmartSpeedController[] {mElevatorMasterWrapper};

        elevatorTransmission = new SensorTransmission(elevatorMotors, mElevatorMasterWrapper);

    //Cargo intake
        mIntakeMaster = new WPI_TalonSRX(8);

        mIntakeMasterWrapper = new TalonSRX(mIntakeMaster);

        intakeMotors = new SmartSpeedController[] {mIntakeMasterWrapper};

        intakeTransmission = new SensorTransmission(intakeMotors, mIntakeMasterWrapper);

    //Hatch mechanism
        gripperPiston = new DoubleSolenoid(2, 3);
        sliderPiston = new DoubleSolenoid(4, 5);
    }
}