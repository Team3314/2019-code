package frc.robot.motion;

import java.io.File;

public class Path {
	
	public enum Mode {
		FORWARD_HIGH,
		FORWARD_LOW,
		BACKWARD_HIGH,
		BACKWARD_LOW
	}
	
	public Path(String name, Mode mode) {
		//XXX TEMPORARY FIX FOR PATHWEAVER LEFT AND RIGHT PATHS SWITCHED
		leftPath = new File("/home/deploy/output/" + name + ".right.pf1.csv");
		rightPath = new File("/home/deploy/output/" + name + ".left.pf1.rcsv");
		this.mode = mode;
	}
	
	private File leftPath;
	private File rightPath;
	private Mode mode;

	public File getLeftPath() {
		return leftPath;
	}
	public File getRightPath() {
		return rightPath;
	}
	public Mode getMode() {
		return mode;
	}
	
}
