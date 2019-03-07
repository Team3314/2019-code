/**
 * CONTROLS
 * 
 * 
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

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

//Actions
	public boolean getAutoGamePiece() {
		return rightStick.getRawButton(2);
	}
	public boolean getAuto() {
		return leftStick.getRawButton(2);
	}
//Drive Controls
    public double getLeftThrottle() {
		double throttle = -leftStick.getRawAxis(1);
		if(Math.abs(throttle) < Constants.kJoystickDeadband)
			throttle = 0;
		return throttle;
	}
	public double getRightThrottle() {
		double throttle = -rightStick.getRawAxis(1);
		if(Math.abs(throttle) < Constants.kJoystickDeadband)
			throttle = 0;
		return throttle;
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
	public boolean getVelocityControl() {
		return rightStick.getRawAxis(3) > .75;
	}
	public boolean getVision() {
		return leftStick.getRawButton(1);
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
		return false;
	}
	public boolean getElevatorLevel1() {
		return gamepad.getPOV() == 180;
	}
	public boolean getElevatorLevel2() {
		return gamepad.getPOV() == 90;
	}
	public boolean getElevatorLevel3() {
		return gamepad.getPOV() == 0 && hasGamepad;
	}
	public boolean getStoreElevatorLevel1() {
		return buttonBox.getRawButton(1);
	}
	public boolean getStoreElevatorLevel2() {
		return buttonBox.getRawButton(4);
	}
	public boolean getStoreElevatorLevel3() {
		return buttonBox.getRawButton(7);
	}
	public boolean getStoreElevatorPickup() {
		return buttonBox.getRawButton(10);
	}
	public boolean getElevatorManual() { 
		return leftStick.getRawAxis(3) >.75;
	
	public double getElevatorSpeed() {
		return -gamepad.getRawAxis(1);
	}
	//Cargo Intake Controls
	public boolean getAutoCargoIntake() {
		return gamepad.getRawButton(1);
	}
	public boolean getCargoIntake() {
		return gamepad.getRawButton(4); // a
	}

	public boolean getCargoTransfer(){
		return gamepad.getRawButton(2); // b
	}
	public boolean getCargoPlace() {
		return gamepad.getRawButton(3); // x
	}
	public boolean getCargoEject() {
		return gamepad.getRawButton(6); // rb
	}
	//Hatch Intake Controls
	public boolean getAutoHatchIntake() {
		return gamepad.getRawButton(9);
	}
	public boolean getAutoHatchPlace() {
		return gamepad.getRawButton(10);
	}
	public boolean getAutoHatchRetract() {
		return gamepad.getRawButton(10);
	}
	public boolean getGripperUp() {
		return gamepad.getRawButton(7); //start
	}
	public boolean getGripperDown() {
		return gamepad.getRawButton(8); //select
	}
	public boolean getSliderIn() {
		return gamepad.getRawButton(5); //lb
	}
	public boolean getSliderOut() {
		return gamepad.getRawAxis(2) > .5; //lt
	}

	public boolean getHome() {
		return false;
	}

	//
	public boolean getClearQueue() {
		return buttonBox.getRawButton(16);
	}
	//Double Hatch Auto 
	public String getLeftRightCenter() {
		if(autoSelector.getRawButton(13)) {
			return "StartL"; // Start Left
		} else if(autoSelector.getRawButton(14)) 
			return "StartR";
		return "StartC"; // Start Right
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
		return buttonBox.getRawButton(5);
	}

	public boolean getPrevious() {
		return buttonBox.getRawButton(6);
	}

	public boolean getAbortClimb() {
		return buttonBox.getRawButton(8);
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

	public boolean getStopAuto() {
		return buttonBox.getRawButton(2); // TODO REPLACE PLACEHOLDER NUMBER
	}
}
