package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.Constants;

public class DriveTrain implements Subsystem {

    private AHRS navx;   

    private WPI_TalonSRX mLeftMaster, mLeftSlave1, mLeftSlave2, mRightMaster, mRightSlave1, mRightSlave2;
	private DoubleSolenoid shifter;

    public enum driveMode {
		IDLE,
		OPEN_LOOP,
		GYROLOCK,
		VELOCITY,
	}

    private static DriveTrain mInstance = new DriveTrain();

    //Control Modes
    private driveMode currentDriveMode = driveMode.OPEN_LOOP;
    ControlMode controlMode = ControlMode.PercentOutput;
    
    //Hardware states
    private boolean mIsHighGear;
	private boolean mIsPTO;
    private NeutralMode neutralMode;

    private double rawLeftSpeed, rawRightSpeed, leftStickInput, rightStickInput, desiredLeftSpeed, desiredRightSpeed, desiredAngle;

    private int leftDrivePositionTicks, rightDrivePositionTicks, leftDriveSpeedTicks, rightDriveSpeedTicks;
    
    private double leftDrivePositionInches, rightDrivePositionInches, leftDriveSpeedRPM, rightDriveSpeedRPM;

    /**
     * @return the mInstance
     */
    public static DriveTrain getInstance() {
        return mInstance;
    }

    private DriveTrain(){
        mLeftMaster = new WPI_TalonSRX(0);
        mLeftSlave1 = new WPI_TalonSRX(1);
        mLeftSlave2 = new WPI_TalonSRX(2);
        mRightMaster = new WPI_TalonSRX(3);
        mRightSlave1 = new WPI_TalonSRX(4);
        mRightSlave2 = new WPI_TalonSRX(5);

        shifter = new DoubleSolenoid(0, 1);

    }

    public void update(){
        if(mIsHighGear) {
    		shifter.set(Constants.kHighGear);
    	}
    	else {
    		shifter.set(Constants.kLowGear);
    	}

    }

    public void outputToSmartDashboard(){

    }   

    public void resetSensors(){

    }

}