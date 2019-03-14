package frc.robot;

import frc.robot.autos.*;
import frc.robot.subsystems.HumanInput;

public class AutoModeSelector {
	private static HumanInput hi = Robot.HI;
	private static String autoModeBinary;
	private static int autoModeDecimal;
	private static Autonomous autoMode;
	private static Autonomous auto0 = null,
						auto1 = new AutoTwoHatchRocketClose(),
						auto2 = new AutoFrontCargoShip(),
						auto3 = null,
						auto4 = null,
						auto5 = null,
						auto6 = null,
						auto7 = null,
						auto8 = null,
						auto9 = null;

	private static Autonomous[] autos = {auto0, auto1, auto2, auto3, auto4, auto5, auto6, auto7, auto8, auto9};

	public static Autonomous getSelectedAutoMode() {
		autoModeBinary = "" + hi.getBinaryEight() + hi.getBinaryFour() + hi.getBinaryTwo() + hi.getBinaryOne();
		autoModeDecimal = Integer.parseInt(autoModeBinary, 2);
		autoMode = autos[autoModeDecimal];
		if(autoMode != null)
			autoMode.reset();
 		return autoMode;
	}
}