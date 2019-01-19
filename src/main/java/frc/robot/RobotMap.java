package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.infrastructure.SensorTransmission;
import frc.robot.infrastructure.SparkMax;
import frc.robot.infrastructure.TalonSRX;

public class RobotMap {

    //Drivetrain
        SparkMax mLeftMaster = new SparkMax(new CANSparkMax(1, Constants.kSparkMotorType));
        SparkMax mLeftSlave1 = new SparkMax(new CANSparkMax(2, Constants.kSparkMotorType));
        SparkMax mLeftSlave2 = new SparkMax(new CANSparkMax(3, Constants.kSparkMotorType));

        SparkMax mRightMaster = new SparkMax(new CANSparkMax(4, Constants.kSparkMotorType));
        SparkMax mRightSlave1 = new SparkMax(new CANSparkMax(5, Constants.kSparkMotorType));
        SparkMax mRightSlave2 = new SparkMax(new CANSparkMax(6, Constants.kSparkMotorType));

        SpeedControllerGroup leftDriveMotors = new SpeedControllerGroup(mLeftMaster, mLeftSlave1, mLeftSlave2);
        SpeedControllerGroup rightDriveMotors = new SpeedControllerGroup(mRightMaster, mRightSlave1, mRightSlave2);

        SensorTransmission leftDrive = new SensorTransmission(leftDriveMotors, mLeftMaster);
        SensorTransmission rightDrive = new SensorTransmission(rightDriveMotors, mRightMaster);


    //Elevator
        TalonSRX mElevatorMaster;

        SpeedControllerGroup elevatorMotors = new SpeedControllerGroup(mElevatorMaster);

        SensorTransmission elevatorTransmission = new SensorTransmission(elevatorMotors, mElevatorMaster);
 
    //Cargo intake
        TalonSRX mIntakeMaster;

        SpeedControllerGroup intakeMotors = new SpeedControllerGroup(mIntakeMaster);

        SensorTransmission intakeTransmission = new SensorTransmission(intakeMotors);

    //Hatch mechanism

}