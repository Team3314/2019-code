package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
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
        PICKUP_EXTEND,
        GRAB,
        EXTEND,
        RETRACT,
        LOWER_AFTER_PICKUP,
        DONE
    }

    private DoubleSolenoid gripper, slider;

    private boolean mIsGripperDown = false;
    private boolean mIsSliderOut = false;

    private Elevator elevator = Robot.elevator;

    private boolean placeRequest, intakeRequest, retractRequest;
    private boolean lastPlaceRequest, lastIntakeRequest, lastRetractRequest;

    private Timer timer = new Timer();

    private State currentState = State.WAITING;

    public HatchMechanism(DoubleSolenoid grip, DoubleSolenoid slide) {
        gripper = grip;
        slider = slide;
    }

    @Override
    public void update() {
        switch(currentState) {
            case WAITING:
                if(intakeRequest && !lastIntakeRequest) {
                    if(!mIsGripperDown) {
                        elevator.set(Constants.kElevatorLoweredHatchPickup);
                        currentState = State.LOWER;
                    }
                    else {
                        elevator.set(Constants.kElevatorHatchPickup);
                        currentState = State.PICKUP_EXTEND; 
                    }
                }
                else if(placeRequest && !lastPlaceRequest) {
                    setSliderOut(true);
                    timer.start();
                    currentState = State.EXTEND;
                }
                else if (retractRequest && !lastRetractRequest) {
                    currentState = State.RETRACT;
                }
                break;
            case LOWER:
                if(elevator.inPosition()) {
                    setGripperDown(true);
                    elevator.set(Constants.kElevatorHatchPickup);
                    currentState = State.PICKUP_EXTEND;
                }
                break;
            case PICKUP_EXTEND:
                if(elevator.inPosition()) {
                    setSliderOut(true);
                    timer.start();
                    currentState = State.GRAB;
                }
                break;
            case GRAB:
                if(timer.get() > .1 && elevator.inPosition()) {
                    setGripperDown(false);
                    timer.stop();
                    timer.reset();
                    currentState = State.RAISE;
                }
                break;
            case RAISE:
                elevator.set(Constants.kElevatorRaisedHatchPickup);
                if(elevator.inPosition()) {
                    setSliderOut(false);
                    currentState = State.LOWER_AFTER_PICKUP;
                }
                break;
            case LOWER_AFTER_PICKUP:
                if(elevator.inPosition()) {
                    currentState = State.DONE;
                }
                break;
            case EXTEND:
                if(timer.get() > .2) {
                    currentState = State.DONE;
                    setGripperDown(true);
                    timer.reset();
                    timer.start();
                }
                break;
            case RETRACT:
                setSliderOut(false);
                setGripperDown(false);
                currentState = State.WAITING;
            case DONE:
                if(!placeRequest && !intakeRequest && !retractRequest) 
                    currentState = State.WAITING;
                break;
        }
        if(mIsSliderOut)
            slider.set(Constants.kSliderOut);
        else
            slider.set(Constants.kSliderIn);    
        if(mIsGripperDown)
            gripper.set(Constants.kGripperDown);
        else   
            gripper.set(Constants.kGripperUp);
        lastPlaceRequest = placeRequest;
        lastIntakeRequest = intakeRequest;

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
        SmartDashboard.putString("Hatch State", currentState.toString());
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

    public void setRetractRequest(boolean request) {
        retractRequest = request;
    }

    @Override
    public void debug() {
        SmartDashboard.putBoolean("Gripper down?", mIsGripperDown);
        SmartDashboard.putBoolean("Slider out?", mIsSliderOut);
    }

}