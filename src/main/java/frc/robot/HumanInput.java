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
		return leftStick.getRawButton(1);
	}
	public boolean getAuto() {
		return leftStick.getRawButton(2);
	}
//Drive Controls
    public double getLeftThrottle() {
		return -leftStick.getRawAxis(1);
	}
	public double getRightThrottle() {
		return -rightStick.getRawAxis(1);
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
		return gamepad.getRawAxis(3) > .5 && gamepad.getPOV() == 180;
	}
	public boolean getElevatorLevel2() {
		return gamepad.getRawAxis(3) > .5 && gamepad.getPOV() == 90;
	}
	public boolean getElevatorLevel3() {
		return gamepad.getRawAxis(3) > .5 && gamepad.getPOV() == 0;
	}
	public boolean getStoreElevatorLevel1() {
		return gamepad.getPOV() == 180;
	}
	public boolean getStoreElevatorLevel2() {
		return gamepad.getPOV() == 90;
	}
	public boolean getStoreElevatorLevel3() {
		return gamepad.getPOV() == 0;
	}
	public boolean getStoreElevatorPickup() {
		return gamepad.getPOV() == 270;
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
	//Hatch Intake Controls
	public boolean getAutoHatchIntake() { //Loading Station (x)
		return gamepad.getRawButton(3);
	}
	public boolean getAutoHatchPlace() {
		return gamepad.getRawButton(5); //lb
	}
	public boolean getAutoHatchRetract() {
		return gamepad.getRawAxis(2) > .5; //lt
	}
	public boolean getGripperUp() {
		return gamepad.getRawButton(4) && buttonBox.getRawButton(11); //First switch on button box and y
	}
	public boolean getGripperDown() {
		return gamepad.getRawButton(1) && buttonBox.getRawButton(11); //First switch on button box and a
	}
	public boolean getSliderIn() {
		return gamepad.getRawButton(3) && buttonBox.getRawButton(11); //First switch on button box and x
	}
	public boolean getSliderOut() {
		return gamepad.getRawButton(2) && buttonBox.getRawButton(11); //First switch on button boc and b
	}

	public boolean getHome() {
		return false;
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
			return "StartR";
		return "StartC"; // Start Right
	}
	public int getBinaryOne() {
		if(autoSelector.getRawButton(1))
			return 1;
		return 0;
	}
	public int getBinaryTwo() {
		if(autoSelector.getRawButton(2))
			return 1;
		return 0;
	}
	public int getBinaryFour() {
		if(autoSelector.getRawButton(3))
			return 1;
		return 0;
	}
	public boolean getClimbMode() {
		return leftStick.getRawButton(5);
	}

	public boolean getAbortClimb() {
		return gamepad.getRawButton(7);
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
}