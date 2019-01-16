package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.infrastructure.SensorTransmission;
import frc.robot.infrastructure.SparkMax;

public class RobotMap {

    //Drivetrain
        SparkMax mLeftMaster, mLeftSlave1, mLeftSlave2, mRightMaster, mRightSlave1, mRightSlave2;

        SpeedControllerGroup leftMotors = new SpeedControllerGroup(mLeftMaster, mLeftSlave1, mLeftSlave2);
        SpeedControllerGroup rightMotors = new SpeedControllerGroup(mRightMaster, mRightSlave1, mRightSlave2);

        SensorTransmission leftDrive = new SensorTransmission(leftMotors, mLeftMaster);


    //Elevator
        WPI_TalonSRX mElevatorMaster;




}