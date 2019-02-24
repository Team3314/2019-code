package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * BASED ON ASHLEIGHS CAD DRAWINGS
 */

public class HatchMechanism implements Subsystem {

    public enum State {
        WAITING,
        LOWER,
        RAISE,
        GRAB,
        EXTEND,
        RETRACT,
        DONE
    }

    private DoubleSolenoid gripper, slider;

    private boolean mIsGripperDown = false;
    private boolean mIsSliderOut = false;

    private boolean placeRequest, intakeRequest;
    private boolean lastPlaceRequest, lastIntakeRequest;

    private State currentState;

    public HatchMechanism(DoubleSolenoid grip, DoubleSolenoid slide) {
        gripper = grip;
        slider = slide;
    }

    @Override
    public void update() {
        switch(currentState) {
            case WAITING:
                break;
            case LOWER:
                break;
            case RAISE:
                break;
            case GRAB:
                break;
            case 
        }

        if (mIsGripperDown) {
            gripper.set(Constants.kGripperDown);
        } else {
            gripper.set(Constants.kGripperUp);
        }

        if (mIsSliderOut) {
            slider.set(Constants.kSliderOut);
        } else {
            slider.set(Constants.kSliderIn);
        }
    }

    /**
     * @param mIsGripperDown the mIsGripperDown to set
     */
    public void setGripperDown(boolean grip) {
        mIsGripperDown = grip;
    }

    /**
     * @param mIsSliderOut the mIsSliderOut to set
     */
    public void setSliderOut(boolean slide) {
        mIsSliderOut = slide;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Gripper down?", mIsGripperDown);
        SmartDashboard.putBoolean("Slider out?", mIsSliderOut);
    }

    @Override
    public void resetSensors() {

    }

    public boolean getIsGripperDown() {
        return mIsGripperDown;
    }
    public boolean getIsSliderOut() {
        return mIsSliderOut;
    }

}