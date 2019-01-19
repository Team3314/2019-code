package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.infrastructure.SensorTransmission;
import frc.robot.infrastructure.SmartSpeedController;
import frc.robot.infrastructure.SparkMax;
import frc.robot.infrastructure.TalonSRX;

public class RobotMap {

    //Drivetrain
        SparkMax mLeftMaster = new SparkMax(new CANSparkMax(1, Constants.kSparkMaxMotorType));
        SparkMax mLeftSlave1 = new SparkMax(new CANSparkMax(2, Constants.kSparkMaxMotorType));
        SparkMax mLeftSlave2 = new SparkMax(new CANSparkMax(3, Constants.kSparkMaxMotorType));

        AHRS navx = new AHRS(SerialPort.Port.kMXP);

        SparkMax mRightMaster = new SparkMax(new CANSparkMax(4, Constants.kSparkMaxMotorType));
        SparkMax mRightSlave1 = new SparkMax(new CANSparkMax(5, Constants.kSparkMaxMotorType));
        SparkMax mRightSlave2 = new SparkMax(new CANSparkMax(6, Constants.kSparkMaxMotorType));

        SmartSpeedController[] leftDriveMotors = {mLeftMaster, mLeftSlave1, mLeftSlave2};
        SmartSpeedController[] rightDriveMotors = {mRightMaster, mRightSlave1, mRightSlave2};

        SensorTransmission leftDrive = new SensorTransmission(leftDriveMotors, mLeftMaster);
        SensorTransmission rightDrive = new SensorTransmission(rightDriveMotors, mRightMaster);


    //Elevator
        TalonSRX mElevatorMaster = new TalonSRX(new WPI_TalonSRX(7));

        SmartSpeedController[] elevatorMotors = {mElevatorMaster};

        SensorTransmission elevatorTransmission = new SensorTransmission(elevatorMotors, mElevatorMaster);

    //Intake
 
        

}