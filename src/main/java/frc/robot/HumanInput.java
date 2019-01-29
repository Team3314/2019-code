
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class HumanInput {

    private static HumanInput mInstance = new HumanInput();
	
	private final Joystick gamepad;
	private final Joystick leftStick;
	private final Joystick rightStick;
	private final Joystick buttonBox;
	private final Joystick autoSelector;
	
	public static HumanInput getInstance() {
		return mInstance;
	}
    
	private HumanInput() {
		gamepad = new Joystick(0);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		buttonBox = new Joystick(3);
		autoSelector = new Joystick(4);

    }

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
	public boolean getElevatorUP() {
		return leftStick.getRawButton(2);
	}
	//TODO buttons for elevator and intakes
	public boolean getElevatorPickup() {
		return false;
	}
	public boolean getElevatorLevel1() {
		return false;
	}
	public boolean getElevatorLevel2() {
		return false;
	}
	public boolean getElevatorLevel3() {
		return false;
	}
	public boolean getElevatorManual() {
		return false;
	}
	public double getElevatorSpeed() {
		return 0;
	}
	public boolean getCargoIntake() {
		return false;
	}
	public boolean getCargoRelease() {
		return false;
	}
	public boolean getGripperUp() {
		return false;
	}
	public boolean getGripperDown() {
		return false;
	}
	public boolean getSliderIn() {
		return false;
	}
	public boolean getSliderOut() {
		return false;
	}
	public boolean getVision() {
		return leftStick.getRawButton(1);
	}
	
}