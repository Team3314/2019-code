package frc.robot;

import frc.robot.autos.*;

public class AutoModeSelector {
	private HumanInput hi = HumanInput.getInstance();
	private String autoModeBinary, delayBinary;
	private int autoModeDecimal, delayDecimal;
	private Autonomous autoMode;
	private Autonomous auto0 = new DoubleHatchAuto(),
						auto1 = new AutoHatchToSideCargo1();

	private Autonomous[] autos = {auto0, auto1}; 
public Autonomous getSelectedAutoMode() {
	autoModeDecimal = Integer.parseInt(autoModeBinary, 2);
		System.out.println(autoModeDecimal);
		autoMode = autos[autoModeDecimal];
		autoMode.reset();
 		return autoMode;
	}
}