package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

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

    private Value gripperPosition = Constants.kGripperUp, sliderPosition = Constants.kSliderIn;

    private Elevator elevator = Robot.elevator;

    private boolean placeRequest, intakeRequest;
    private boolean lastPlaceRequest, lastIntakeRequest;

    private Timer timer = new Timer();

    private State currentState = State.WAITING;

    public HatchMechanism(DoubleSolenoid grip, DoubleSolenoid slide) {
        gripper = grip;
        slider = slide;
    }

    @Override
    public void update() {
        if((!placeRequest && lastPlaceRequest) ||
            (!intakeRequest && lastIntakeRequest)) {
                currentState = State.WAITING;
                timer.reset();
            }
        switch(currentState) {
            case WAITING:
                if(intakeRequest && !lastIntakeRequest) {
                    currentState = State.LOWER; 
                }
                else if(placeRequest && !lastPlaceRequest) {
                    currentState = State.GRAB;
                }
                break;
            case LOWER:
                if(elevator.inPosition()) {
                    setGripperDown(true);
                    currentState = State.RAISE;
                }
                break;
            case RAISE:
                elevator.set(Constants.kElevatorRaisedHatchPickup);
                if(elevator.inPosition()) {
                    currentState = State.GRAB;
                    timer.start();
                }
                break;
            case GRAB:
                setGripperDown(false);
                if(timer.get() > .1) {    
                    timer.stop();
                    timer.reset();
                    currentState = State.DONE;
                }
                break;
            case EXTEND:
                if(timer.get() > .1) {
                    currentState = State.RETRACT;
                    setSliderOut(false);
                    timer.reset();
                }
                break;
            case RETRACT:
                if(timer.get() > .1) {
                    currentState = State.DONE;
                    setGripperDown(false);
                    timer.reset();
                    timer.stop();
                }
            case DONE:
                break;
        }
        slider.set(sliderPosition);
        gripper.set(gripperPosition);
    }

    /**
     * @param mIsGripperDown the mIsGripperDown to set
     */
    public void setGripperDown(boolean grip) {
        if(grip) {
            gripperPosition = Constants.kGripperDown;
        }
        else {
            gripperPosition = Constants.kGripperUp;
        }
    }

    /**
     * @param mIsSliderOut the mIsSliderOut to set
     */
    public void setSliderOut(boolean slide) {
        if(slide) {
           sliderPosition = Constants.kSliderOut; 
        }
        else {
            sliderPosition = Constants.kSliderIn;
        }
    }

    /**
     * @param intakeRequest the intakeRequest to set
     */
    public void setIntakeRequest(boolean intakeRequest) {
        this.intakeRequest = intakeRequest;
    }
    
     /**
     * @param placeRequest the placeRequest to set
     */
    public void setPlaceRequest(boolean placeRequest) {
        this.placeRequest = placeRequest;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Gripper down?", mIsGripperDown);
        SmartDashboard.putBoolean("Slider out?", mIsSliderOut);
    }

    @Override
    public void resetSensors() {

    }

    public boolean isDone() {
        return currentState == State.DONE;
    }

    public boolean getIsGripperDown() {
        return mIsGripperDown;
    }
    public boolean getIsSliderOut() {
        return mIsSliderOut;
    }

}