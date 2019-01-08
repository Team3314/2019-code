
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

}