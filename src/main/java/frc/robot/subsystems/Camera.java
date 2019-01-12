package frc.robot.subsystems;

public class Camera implements Subsystem {

    private static Camera mInstance = new Camera();
	
	public static Camera getInstance() {
		return mInstance;
	}

	@Override
	public void update() {

	}

	@Override
	public void outputToSmartDashboard() {

	}

	@Override
	public void resetSensors() {

	}

	public double getSteeringAdjust() {
		return 0;
	}

}