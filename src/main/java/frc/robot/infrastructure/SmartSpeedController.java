package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;

public abstract class SmartSpeedController implements SpeedController, CustomEncoder {

    SpeedController controller;
    boolean isInverted;
    double distancePerPulse;
    PIDSourceType pidSource;

    @Override
    public void stopMotor() {
        controller.stopMotor();
    }

    @Override
    public void disable() {
        controller.disable();
    }

    @Override
    public boolean getInverted() {
        return controller.getInverted();
    }

    @Override
    public void setInverted(boolean setInverted) {
        isInverted = setInverted;
    }

    @Override
    public double get() {
        return controller.get();
    }

    @Override
    public void set(double speed) {
        controller.set(speed);
    }

    @Override
    public void setDistancePerPulse(double distancePerPulse) {
        this.distancePerPulse = distancePerPulse;
    }
    @Override
    public double getDistance() {
        return getEncoderCounts() * distancePerPulse;
    } 
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSource = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSource;
    }

    @Override
    public double pidGet() {
        if(pidSource == PIDSourceType.kDisplacement)
            return getDistance();
        else if(pidSource == PIDSourceType.kRate)
            return getVelocity();
        else
            return -1;
	}

    public abstract void set(double speed, SpeedControllerMode mode);

    public abstract double getOutputCurrent();

    public abstract double getOutputVoltage();

    public abstract void setIdleMode(frc.robot.infrastructure.IdleMode mode);

    public abstract void setControlMode(frc.robot.infrastructure.SpeedControllerMode mode);

    public abstract void setRampRate(double rate);
}