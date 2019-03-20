package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
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

		
	public DriverStation ds = DriverStation.getInstance();

		
	private NetworkTableEntry matchNumber = table.getEntry("Match Number"), 
	matchTime = table.getEntry("Match Time"),
    enabled = table.getEntry("Enabled");
		
	private boolean targetInView, hasRight, hasLeft;
	private double targetHorizError;

	private boolean lightRingsOn = true;

	private double correction;

	private Solenoid leftLightRing, rightLightRing, targetsLight;
    
    private double distance, highDistance;
	
	public Camera(Solenoid leftLightRing, Solenoid rightLightRing, Solenoid targetsLight) {
		this.leftLightRing = leftLightRing;
		this.rightLightRing = rightLightRing;
		this.targetsLight = targetsLight;
	}	

	@Override
    public void update() {
		leftLightRing.set(lightRingsOn);
		rightLightRing.set(lightRingsOn);

        
		targetHorizError = -(table.getEntry("Angle To Target").getDouble(0));
		distance = table.getEntry("Distance").getDouble(1337.254);
		highDistance = table.getEntry("DistanceHigh").getDouble(1337.254);
		hasLeft = table.getEntry("Left hasRight").getBoolean(false);
		hasRight = table.getEntry("Left hasLeft").getBoolean(false);
		
		enabled.setBoolean(ds.isEnabled());
		matchNumber.setNumber(ds.getMatchNumber());
		matchTime.setNumber(ds.getMatchTime());

		targetInView = hasLeft && hasRight &&  distance >= 24;

		targetsLight.set(targetInView);
    }

	/**
	 * Getters
	 */

    /**
	 * 
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
        return distance;
	}

	public double getHighDistance() {
		return highDistance;
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
		SmartDashboard.putBoolean("Targets in view", isTargetInView());
		SmartDashboard.putNumber("Target horizontal error", getTargetHorizError());
		SmartDashboard.putNumber("Distance", getDistance());
		SmartDashboard.putBoolean("Light Rings On", getLightRingsOn());
	}

	@Override
	public void resetSensors() {
	}

	@Override
	public void debug() {
	}
}