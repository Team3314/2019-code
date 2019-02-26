package frc.robot;

import frc.robot.autos.*;

public class AutoModeSelector {
	private static HumanInput hi = Robot.HI;
	private static String autoModeBinary;
	private static int autoModeDecimal;
	private static Autonomous autoMode;
	private static Autonomous auto0 = new DoubleHatchAuto(),
						auto1 = new AutoHatchToSideCargo1();

	private static Autonomous[] autos = {auto0, auto1};

	public static Autonomous getSelectedAutoMode() {
		autoModeBinary = "" + hi.getBinaryFour() + hi.getBinaryTwo() + hi.getBinaryOne();
		autoModeDecimal = Integer.parseInt(autoModeBinary, 2);
		autoMode = autos[autoModeDecimal];
		autoMode.reset();
 		return autoMode;
	}
}