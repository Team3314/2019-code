package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Camera
 */
public class Camera implements Subsystem {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double targetInView, targetHorizError, targetVertError, targetArea, targetSkew, targetLatency;
	
	private double ledMode, camMode, snapshot;
	private String ledString, camString, snapshotString;

	private double correction;

	private double turnAngle;
    
    private double rawDistance;
    private double targetHeight = 0; //31.40 or 39.25 inches off the carpet

	@Override
    public void update() {
		if (camMode == Constants.kVisionProcessorMode) {
			setLEDMode(Constants.kLEDDefault);
		} else {
			setLEDMode(Constants.kLEDOff);
		}

        targetInView = table.getEntry("tv").getDouble(0);
        targetHorizError = table.getEntry("tx").getDouble(-1337.254); //deg
        targetVertError = table.getEntry("ty").getDouble(-1337.254); //deg
		targetArea = table.getEntry("ta").getDouble(0);
		targetSkew = table.getEntry("ts").getDouble(0);
		targetLatency = 11 + table.getEntry("tl").getDouble(0);

		correction = targetHorizError * Constants.kVisionCtrl_kP;

        rawDistance = (targetHeight - Constants.kCameraHeight) / (Math.tan(Math.toRadians(targetVertError + Constants.kMountingAngle)));
        //solution to height differentiation problem: different pipelines with same settings? pipeline 0 = ball target height, pipeline 1 = the 3 hatch target heights;
    }

	/**
	 * Getters
	 */

    /**
     * @return the targetsInView
     */
    public boolean isTargetInView() {
        if (targetInView == 1.0) {
            return true;
        } else {
            return false;
        }
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
    public double getRawDistance() {
        return rawDistance;
	}

	public String getLEDMode() {
		if (ledMode == Constants.kLEDDefault) {
			ledString = "DEFAULT";
		} else if (ledMode == Constants.kLEDOff) {
			ledString = "OFF";
		} else if (ledMode == Constants.kLEDOn) {
			ledString = "ON";
		}
		return ledString;
	}
	
	public String getCamMode() {
		if (camMode == Constants.kVisionProcessorMode) {
			camString = "VISION PROCESSOR";
		} else if (camMode == Constants.kDriverCameraMode) {
			camString = "DRIVER CAMERA";
		}
		return camString;
	}
	
	public String getSnapshot() {
		if (snapshot == Constants.kSnapshotOff) {
			snapshotString = "OFF";
		} else if (snapshot == Constants.kSnapshotOn) {
			snapshotString = "ON";
		}
		return snapshotString;
	}

	/**
	 * @return the correction
	 */
	public double getCorrection() {
		return correction;
	}

	public double getTurnAngle() {
		return turnAngle;
	}

	/**
	 * Setters
	 */
	public void setLEDMode(int ledMode) {
	 table.getEntry("ledMode").setDouble(ledMode);
	}
	
	public void setCamMode(int camMode) {
	 table.getEntry("camMode").setDouble(camMode);
	}
	
	public void setSnapshot(int snapshot) {
	 table.getEntry("snapshot").setDouble(snapshot);
	}
	
	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("Targets in view", isTargetInView());
		SmartDashboard.putNumber("Target horizontal error", getTargetHorizError());
		SmartDashboard.putNumber("Target vertical error", targetVertError);
		SmartDashboard.putNumber("Target area", targetArea);
		SmartDashboard.putNumber("Target skew", targetSkew);
		SmartDashboard.putNumber("Target latency", targetLatency);
		SmartDashboard.putNumber("Raw distance", getRawDistance());
		SmartDashboard.putString("LED mode", getLEDMode());
		SmartDashboard.putString("Camera mode", getCamMode());
		SmartDashboard.putString("Snapshot mode", getSnapshot());
		SmartDashboard.putNumber("Correction", correction);
	}

	@Override
	public void resetSensors() {
	}
}