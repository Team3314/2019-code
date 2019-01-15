package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class RobotMap {

    //Drivetrain
        CANSparkMax mLeftMaster, mLeftSlave1, mLeftSlave2, mRightMaster, mRightSlave1, mRightSlave2;

        SpeedControllerGroup leftDrive = new SpeedControllerGroup(mLeftMaster, mLeftSlave1, mLeftSlave2);
        SpeedControllerGroup RightDrive = new SpeedControllerGroup(mRightMaster, mRightSlave1, mRightSlave2);


    //Elevator
        WPI_TalonSRX mElevatorMaster;




}