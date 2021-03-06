package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Transmission extends SendableBase implements SpeedController {

    private boolean m_isInverted;
    protected SmartSpeedController[] motors;
    protected static int instances;

    public Transmission(SmartSpeedController[] group) {
        motors = group;
    }

    @Override
    public void set(double speed) {
        motors[0].set(speed);
    }


    public void set(double speed, SpeedControllerMode mode) {
        if(m_isInverted)
            motors[0].set(-speed, mode);
        else
            motors[0].set(speed,mode);

    }

    
    public void set(double speed, SpeedControllerMode mode, double arbitraryFeedForward) {
        if(m_isInverted)
            motors[0].set(-speed, mode, -arbitraryFeedForward);
        else
            motors[0].set(speed,mode, arbitraryFeedForward);
    }

    @Override
    public double get() {
        if (motors.length > 0) {
            return motors[0].get();
        }
        return 0.0;
    }

    public void setIdleMode(IdleMode mode) {
        for (SmartSpeedController speedController : motors) {
            speedController.setIdleMode(mode);
        }
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_isInverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return m_isInverted;
    }

    @Override
    public void disable() {
        for (SmartSpeedController speedController : motors) {
            speedController.disable();
        }
    }

    @Override
    public void stopMotor() {
        for (SmartSpeedController speedController : motors) {
            speedController.stopMotor();
        }
    }

    @Override
    public void pidWrite(double output) {
        set(output);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Speed Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    public double getOutputVoltage() {
        return motors[0].getOutputVoltage();
    }

    public double getOutputCurrent(int motorID) {
        if(motorID < motors.length) {
            return motors[motorID].getOutputCurrent();
        }
        else {
            return -1;
        }
    }


}