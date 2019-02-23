/**
 * CONTROLS
 * 
 * 
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class HumanInput {
	
	private final Joystick gamepad;
	private final Joystick leftStick;
	private final Joystick rightStick;
	private final Joystick buttonBox;
	private final Joystick autoSelector;
    
	public HumanInput() {
		gamepad = new Joystick(0);
		leftStick = new Joystick(1);

		rightStick = new Joystick(2);
		buttonBox = new Joystick(3);
		autoSelector = new Joystick(4);

	}

//Actions
	public boolean getAutoGamePiece() {
		return false;//rightStick.getRawButton(4);
	}
	public int getElevatorPlaceLevel() {
		return 0;
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
		return rightStick.getRawButton(2);
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
	//TODO buttons
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
		return gamepad.getPOV() == 0;
	}
	public boolean getElevatorManual() {
		return leftStick.getRawAxis(3) >.75;//buttonBox.getRawButton(1);
	}
	public double getElevatorSpeed() {
		return gamepad.getRawAxis(1);
	}
	//Cargo Intake Controls
	public boolean getAutoCargoIntake() {
		return gamepad.getRawButton(1);
	}
	public boolean getCargoIntake() {
		return gamepad.getRawButton(4); // a
	}

	public boolean getCargoTransfer() {
		return gamepad.getRawButton(2); // b
	}
	public boolean getCargoPlace() {
		return gamepad.getRawButton(3); // x
	}
	public boolean getCargoEject() {
		return gamepad.getRawButton(6); // rb
	}
	public boolean getCargoStopDown() {
		return gamepad.getRawButton(10); // right stick
	}
	//Hatch Intake Controls
	public boolean getAutoHatchIntake() {
		return gamepad.getRawAxis(3) > .75;
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

	public boolean getClimb() {
		return leftStick.getRawButton(7) && leftStick.getRawButton(11);
	}
	public boolean getNotClimb() {
		return rightStick.getRawButton(7) && rightStick.getRawButton(11);
	}

	public boolean getRaiseBack() {
		return gamepad.getRawButton(9);
	}

	public boolean getRaiseFront() {
		return leftStick.getRawButton(7) && leftStick.getRawButton(11);
	}
	public boolean getClimbMode() {
		return false;
	}

	public boolean getCargoClimb() {
		return gamepad.getRawButton(9);
	}

	public boolean getAbortClimb() {
		return true;
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
}