package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Camera
 */
public class Camera implements Subsystem {

	public enum DSCamera {
		FRONT,
		BACK,
		LEFT,
		RIGHT
	}

	private NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("jetson");
	private NetworkTableEntry expectedTargetAngle = new NetworkTableEntry(NetworkTableInstance.getDefault(), 420), 
		driveStationCamera = new NetworkTableEntry(NetworkTableInstance.getDefault(), 420);
		
	private boolean targetInView, leftHasLeft, rightHasRight;
	private double targetHorizError;

	private boolean lightRingsOn = true;

	private double correction;

	private Solenoid leftLightRing, rightLightRing, targetsLight;
    
    private double hatchDistance, cargoDistance;
	
	public Camera(Solenoid leftLightRing, Solenoid rightLightRing, Solenoid targetsLight) {
		this.leftLightRing = leftLightRing;
		this.rightLightRing = rightLightRing;
		this.targetsLight = targetsLight;
	}	

	@Override
    public void update() {
		leftLightRing.set(lightRingsOn);
		rightLightRing.set(lightRingsOn);

        
		targetHorizError = -table.getEntry("Angle To Target").getDouble(0);
		hatchDistance = table.getEntry("Distance").getDouble(1337.254);
		rightHasRight = table.getEntry("Right hasRight").getBoolean(false);
		leftHasLeft = table.getEntry("Left hasLeft").getBoolean(false);


		targetInView = rightHasRight && leftHasLeft &&  hatchDistance >= 24;

		targetsLight.set(targetInView);
    }

	/**
	 * Getters
	 */

    /**
     * @return the targetsInView
     */
    public boolean isTargetInView() {
        return targetInView;
    }

    /**
     * @return the targetHorizOffset
     */
    public double getTargetHorizError() {
		return targetHorizError;
	}

	/**
     * @return the rawDistance
     */
    public double getDistance() {
        return hatchDistance;
	}

	public boolean getLightRingsOn() {
		return lightRingsOn;
	}

	/**
	 * @return the correction
	 */
	public double getCorrection() {
		return correction;
	}

	/**
	 * Setters
	 */
	public void setLightRings(boolean on) {
		lightRingsOn = on;
	}
	
	public void setCamMode(int camMode) {
	 table.getEntry("camMode").setDouble(camMode);
	}
	
	public void setSnapshot(int snapshot) {
	 table.getEntry("snapshot").setDouble(snapshot);
	}

	public void setDSView(DSCamera camera) {
		driveStationCamera.setString(camera.toString());
	}

	public void setHint(String hint) {
		expectedTargetAngle.setString(hint);
	}
	
	@Override
	public void outputToSmartDashboard() {
	}

	@Override
	public void resetSensors() {
	}

	@Override
	public void debug() {
		SmartDashboard.putBoolean("Targets in view", isTargetInView());
		SmartDashboard.putNumber("Target horizontal error", getTargetHorizError());
		SmartDashboard.putNumber("Distance", getDistance());
		SmartDashboard.putBoolean("Light Rings On", getLightRingsOn());
	}
}