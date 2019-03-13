/**
 * CONTROLS
 * 
 * 
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

// TODO: consider making HumanInput a subsystem (specifically so it is updatable)...
// then move things like managing hasGamepad into update. (possibly check other devices as well) 
// also anything that might be better done once, could be done there. 
// TODO: consider making a class that represents a button and handles things...
// like rising and falling edge detection. 
// TODO: consider making a class that represents a POV or at least a getPOV method...
// that accounts for hasGamepad (or whichever device that POV resides on)
// TODO: consider getAxis function or Axis class with improved functionallity possibly by...
// managing deadband in a way that leaves deadband at values just off zero.
// providing scaling or data shaping (crazy: out = a*in^3+b*in^2+c*in+d) or just scaling
// reversing, etc. 

public class HumanInput {
	
	public final Joystick gamepad;
	private final Joystick leftStick;
	private final Joystick rightStick;
	private final Joystick buttonBox;
	private final Joystick autoSelector;

	public boolean hasGamepad = false;
    
	public HumanInput() {
		gamepad = new Joystick(0);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		buttonBox = new Joystick(3);
		autoSelector = new Joystick(4);
	}

	public boolean getShift() {
		return gamepad.getRawAxis(3) >= .75;
	}
	//Actions
	public boolean getAutoGamePiece() {
		return leftStick.getRawButton(1); // trigger
	}
	public boolean getAuto() {
		return false;//leftStick.getRawButton(2); // side button
	}
	public boolean getStopAuto() {
		return gamepad.getRawButton(10); // right stick in
	}
	public boolean getGamePieceInteract() {
		return gamepad.getRawButton(6);
	}
//Drive Controls
   public double getLeftThrottle() {
		double throttle = -leftStick.getRawAxis(1);
		if(Math.abs(throttle) < Constants.kJoystickDeadband)
			throttle = 0;
		else if(throttle < 0) {
			throttle += Constants.kJoystickDeadband;
		}
		else if(throttle > 0) {
			throttle -= Constants.kJoystickDeadband;
		}
		throttle *= Constants.kJoystickThrottleScale;
		return throttle;
	}
	public double getRightThrottle() {
		double throttle = -rightStick.getRawAxis(1);
		if(Math.abs(throttle) < Constants.kJoystickDeadband)
			throttle = 0;
		else if(throttle < 0) {
			throttle += Constants.kJoystickDeadband;
		}
		else if(throttle > 0) {
			throttle -= Constants.kJoystickDeadband;
		}
		throttle *= Constants.kJoystickThrottleScale;
		return (throttle);
	}
	public boolean getLowGear() {
		return leftStick.getRawButton(4);
	}
	public boolean getHighGear() {
		return leftStick.getRawButton(6);
	}
	public boolean getGyrolock() {
		return rightStick.getRawButton(1);
	}
	public boolean getVision() {
		return false;
	}
	public boolean getLightRingsOff() {
		return leftStick.getRawButton(9);
	}
	public boolean getLightRingsOn() {
		return leftStick.getRawButton(10);
	}
	public boolean getResetGyro() {
		return buttonBox.getRawButton(9);
	}
	//Elevator Controls
	public boolean getElevatorPickup() {
		return getShift() && getStoreElevatorPickup();
	}
	public boolean getElevatorLevel1() {
		return getShift() && gamepad.getPOV() == 180;
	}
	public boolean getElevatorLevel2() {
		return getShift() && gamepad.getPOV() == 90;
	}
	public boolean getElevatorLevel3() {
		return getShift() && gamepad.getPOV() == 0;
	}
	public boolean getElevatorCargoShip() {
		return getShift() && getStoreCargoShip();
	}
	public boolean getStoreElevatorLevel1() {
		return gamepad.getPOV() == 180;
	}
	public boolean getStoreElevatorLevel2() {
		return gamepad.getPOV() == 90;
	}
	public boolean getStoreElevatorLevel3() {
		return gamepad.getPOV() == 0 && hasGamepad;
	}
	public boolean getStoreElevatorPickup() {
		return gamepad.getPOV() == 270;
	}
	public boolean getStoreCargoShip() {
		return gamepad.getRawButton(9);
	}
	public boolean getStoreBallStationPickup() {
		return gamepad.getRawButton(7);
	}
	public boolean getElevatorManual() {
		return buttonBox.getRawButton(11); // left switch on button box
	}
	public double getElevatorSpeed() {
		return -gamepad.getRawAxis(1);
	}
	//Cargo Intake Controls
	public boolean getAutoCargoIntake() {
		return gamepad.getRawButton(1); // a
	}
	public boolean getIntakeFurtherDown() {
		return getShift() && getAutoCargoIntake();
	}
	public boolean getCargoIntake() {
		return buttonBox.getRawButton(1); // button box 1 
	}

	public boolean getCargoTransfer(){
	return buttonBox.getRawButton(2); // button box 2
	}
	public boolean getCargoPlace() {
		return gamepad.getRawButton(2); // b
	}
	public boolean getCargoEject() {
		return buttonBox.getRawButton(3); // Button Box 3 shoots cargo backwards towards intake
	}
	public boolean getCargoStationPickup() {
		return gamepad.getRawButton(4);
	}
	public boolean getCargoReverseOuttake() {
		return buttonBox.getRawButton(4);
	}
	//Hatch Intake Controls
	public boolean getHatchIntake() { //Loading Station (x)
		return gamepad.getRawButton(3);
	}
	public boolean getHatchPlace() {
		return gamepad.getRawButton(5); //lb
	}
	public boolean getHatchRetract() {
		return gamepad.getRawAxis(2) > .5; //lt
	}
	public boolean getGripperUp() {
		return gamepad.getRawButton(4) && buttonBox.getRawButton(12); //Second switch on button box and y
	}
	public boolean getGripperDown() {
		return gamepad.getRawButton(1) && buttonBox.getRawButton(12); //Second switch on button box and a
	}
	public boolean getSliderIn() {
		return gamepad.getRawButton(3) && buttonBox.getRawButton(12); //Second switch on button box and x
	}
	public boolean getSliderOut() {
		return gamepad.getRawButton(2) && buttonBox.getRawButton(12); //Second switch on button boc and b
	}

	public boolean getHome() {
		return buttonBox.getRawButtonPressed(10);
	}

	//
	public boolean getClearQueue() {
		return false;				//buttonBox.getRawButton(16);
	}
	//Double Hatch Auto 
	public String getLeftRightCenter() {
		if(autoSelector.getRawButton(13)) {
			return "StartL"; // Start Left
		} else if(autoSelector.getRawButton(14)) 
			return "StartR";// Start Right
		return "StartC"; 
	}
	public int getBinaryOne() {
		if(buttonBox.getRawButton(13))
			return 1;
		return 0;
	}
	public int getBinaryTwo() {
		if(buttonBox.getRawButton(14))
			return 1;
		return 0;
	}
	public int getBinaryFour() {
		if(buttonBox.getRawButton(15))
			return 1;
		return 0;
	}
	public int getBinaryEight() {
		if(buttonBox.getRawButton(16))
			return 1;
		return 0;
	}
	public boolean getClimbMode() {
		return gamepad.getRawButton(7) && gamepad.getRawButton(8);
	}

	public boolean getPrevious() {
		return gamepad.getRawButton(7) && gamepad.getRawButton(6);
	}

	public boolean getAbortClimb() {
		return buttonBox.getRawButton(6);
	}

//Camera
	public boolean getForwardCamera() {
		return rightStick.getPOV() == 0;
	}
	public boolean getRightCamera() {
		return rightStick.getPOV() == 90;
	}
	public boolean getBackCamera() {
		return rightStick.getPOV() == 180;
	}
	public boolean getLeftCamera() {
		return rightStick.getPOV() == 270;
	}

	public boolean turnToZero() {
		return leftStick.getPOV() == 0;
	}
	public boolean turnToRight() {
		return leftStick.getPOV() == 90;
	}
	public boolean turnBack() {
		return leftStick.getPOV() == 180;
	}
	public boolean turnToLeft() {
		return leftStick.getPOV() == 270;
	}
	public boolean getHasGamepad() {
		return gamepad.getPOV() == -1;
	}
	public void setHasGamepad(boolean gamepad) {
		hasGamepad = gamepad;
	}

	public boolean getDebugMode() {
		return autoSelector.getRawButton(1);
	}

}
