package frc.robot;

import frc.robot.autos.*;

public class AutoModeSelector {
	private HumanInput hi = HumanInput.getInstance();
	private String autoModeBinary, delayBinary;
	private int autoModeDecimal, delayDecimal;
	private Autonomous autoMode;
	
	public Autonomous getSelectedAutoMode() {
		autoModeDecimal = Integer.parseInt(autoModeBinary, 2);
		autoMode.reset();
 		return autoMode;
	}
}