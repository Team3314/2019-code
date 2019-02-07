package frc.robot;

import frc.robot.autos.*;

public class AutoModeSelector {
	private HumanInput hi = Robot.HI;
	private String autoModeBinary, delayBinary;
	private int autoModeDecimal, delayDecimal;
	private Autonomous autoMode;
	private Autonomous auto0 = new DoubleHatchAuto(),
						auto1 = new AutoHatchToSideCargo1();

	private Autonomous[] autos = {auto0, auto1};
public Autonomous getSelectedAutoMode() {
		autoModeDecimal = Integer.parseInt(autoModeBinary, 2);
		autoMode.reset();
 		return autoMode;
	}
}