package frc.robot;

import edu.wpi.first.wpilibj.PIDOutput;

/*Takes the output of the gyro PID controller and creates a variable from it which is used to speed up one side and slow down the other
to keep the robot driving straight in gyrolock. */

public class CustomPIDOutput implements PIDOutput {
	public double output;
	
	@Override
	public void pidWrite(double output) {
		this.output = output;
	}
	public double getOutput() {
		return output;
	}
}