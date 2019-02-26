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
        RELEASE,
        RETRACT
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
        switch(currentState) {
            case WAITING:
                if(intakeRequest && !lastIntakeRequest) {
                    if(!mIsGripperDown) {
                        elevator.set(Constants.kElevatorBallLevel1);
                        currentState = State.LOWER;
                    }
                    else {
                        elevator.set(Constants.kElevatorHatchPickup);
                        timer.start();
                        currentState = State.GRAB; 
                    }
                }
                else if(placeRequest && !lastPlaceRequest) {
                    setSliderOut(true);
                    timer.start();
                    currentState = State.EXTEND;
                }
                break;
            case LOWER:
                if(elevator.inPosition()) {
                    setGripperDown(true);
                    timer.start();
                    elevator.set(Constants.kElevatorHatchPickup);
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
                    currentState = State.WAITING;
                    timer.start();
                }
                break;
            case EXTEND:
                if(timer.get() > .2) {
                    currentState = State.RELEASE;
                    setGripperDown(true);
                    timer.reset();
                    timer.start();
                }
                break;
            case RELEASE:
                if(placeRequest && !lastPlaceRequest) {
                    currentState = State.RETRACT;
                }
                break;
            case RETRACT:
                setSliderOut(false);
                setGripperDown(false);
                currentState = State.WAITING;
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
        SmartDashboard.putBoolean("Gripper down?", mIsGripperDown);
        SmartDashboard.putBoolean("Slider out?", mIsSliderOut);
    }

    @Override
    public void resetSensors() {

    }

    public boolean isWaiting() {
        return currentState == State.WAITING;
    }

    public boolean getIsGripperDown() {
        return mIsGripperDown;
    }
    public boolean getIsSliderOut() {
        return mIsSliderOut;
    }

}