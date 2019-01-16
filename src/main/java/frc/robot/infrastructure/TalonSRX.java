package frc.robot.infrastructure;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonSRX extends SmartSpeedController {

    WPI_TalonSRX talon;

    public TalonSRX(WPI_TalonSRX talon) {
        this.talon = talon;
    }

    @Override
    public void pidWrite(double output) {

    }

    @Override
    public void set(double speed) {

    }

    @Override
    public double get() {
        return 0;
    }

    @Override
    public void setInverted(boolean isInverted) {

    }

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public void disable() {

    }

    @Override
    public void stopMotor() {

    }

    @Override
    public int getEncoderCounts() {
        return 0;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

}