package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * BASED ON ASHLEIGHS CAD DRAWINGS
 */

public class HatchMechanism implements Subsystem {

    private static HatchMechanism mInstance = new HatchMechanism();

    public static HatchMechanism getInstance() {
        return mInstance;
    }

    private DoubleSolenoid gripperPiston;
    private DoubleSolenoid sliderPiston;

    private boolean mIsGripperDown = false;
    private boolean mIsSliderOut = false;

    private HatchMechanism() {
        gripperPiston = new DoubleSolenoid(2, 3);
        sliderPiston = new DoubleSolenoid(4, 5);
    }

    @Override
    public void update() {
        if (mIsGripperDown) {
            gripperPiston.set(Constants.kGripperDown);
        } else {
            gripperPiston.set(Constants.kGripperUp);
        }

        if (mIsSliderOut) {
            sliderPiston.set(Constants.kSliderOut);
        } else {
            sliderPiston.set(Constants.kSliderIn);
        }
    }

    /**
     * @param mIsGripperDown the mIsGripperDown to set
     */
    public void setGripperDown(boolean gripper) {
        mIsGripperDown = gripper;
    }

    /**
     * @param mIsSliderOut the mIsSliderOut to set
     */
    public void setSliderOut(boolean slider) {
        mIsSliderOut = slider;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Gripper down?", mIsGripperDown);
        SmartDashboard.putBoolean("Slider out?", mIsSliderOut);
    }

    @Override
    public void resetSensors() {

    }

}