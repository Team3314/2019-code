package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.SpeedController;

public abstract class SmartSpeedController implements SpeedController, CustomEncoder {

    public void stopMotor() {
        this.stopMotor();
    }

    public void disable() {
        this.disable();
    }

    public boolean getInverted() {
        return this.getInverted();
    }

    public void setInverted(boolean setInverted) {
        this.setInverted(setInverted);
    }

    public double get() {
        return this.get();
    }

    @Override
    public void set(double speed) {
        this.set(speed);
    }

}