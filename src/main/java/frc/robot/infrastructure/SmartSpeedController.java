package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.SpeedController;

public abstract class SmartSpeedController implements SpeedController, CustomEncoder {

    @Override
    public void stopMotor() {
        this.stopMotor();
    }

    @Override
    public void disable() {
        this.disable();
    }

    @Override
    public boolean getInverted() {
        return this.getInverted();
    }

    @Override
    public void setInverted(boolean setInverted) {
        this.setInverted(setInverted);
    }

    @Override
    public double get() {
        return this.get();
    }

    @Override
    public void set(double speed) {
        this.set(speed);
    }
    public abstract void set(double speed, SpeedControllerMode mode);

    public abstract double getOutputCurrent();

    public abstract double getOutputVoltage();

    public abstract void setIdleMode(frc.robot.infrastructure.IdleMode mode);

    public abstract void setControlMode(frc.robot.infrastructure.SpeedControllerMode mode);

}