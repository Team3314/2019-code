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
		leftPath = new File("/home/lvuser/paths/" + name + "_left_detailed.csv");
		rightPath = new File("/home/lvuser/paths/" + name + "_right_detailed.csv");
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
