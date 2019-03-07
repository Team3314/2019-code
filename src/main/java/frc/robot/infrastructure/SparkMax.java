package frc.robot.infrastructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

public class SparkMax extends SmartSpeedController {

    ControlType mode;
    int pidSlot;

    public SparkMax(CANSparkMax spark) {
        super.controller = spark;
    }

    @Override
    public void set(double speed, SpeedControllerMode mode) {
        setControlMode(mode);
        if(isInverted)
            ((CANSparkMax)controller).getPIDController().setReference(-speed, this.mode, pidSlot);
        else
            ((CANSparkMax)controller).getPIDController().setReference(speed, this.mode, pidSlot);
    }

    @Override
    public void set(double speed, SpeedControllerMode mode, double arbitraryFeedForward) {
        setControlMode(mode);
        if(isInverted)
            ((CANSparkMax)controller).getPIDController().setReference(-speed, this.mode, pidSlot, arbitraryFeedForward);
        else
            ((CANSparkMax)controller).getPIDController().setReference(speed, this.mode, pidSlot, arbitraryFeedForward);
    }

    public CANSparkMax getController() {
        return ((CANSparkMax)controller);
    }

    @Override
    public void pidWrite(double output) {
        ((CANSparkMax)controller).set(output);
    }

    @Override
    public double getEncoderCounts() {
        return (Math.round(((CANSparkMax)controller).getEncoder().getPosition() * 42));
    }

    @Override
    public double getVelocity() {
        return ((CANSparkMax)controller).getEncoder().getVelocity() * 42 / 60;
    }

    @Override
    public void zero() {
      ((CANSparkMax)controller).setEncPosition(0);
    }
    @Override
    public void setDistancePerPulse(double distancePerPulse) {
        ((CANSparkMax)controller).getEncoder().setPositionConversionFactor(distancePerPulse);
        ((CANSparkMax)controller).getEncoder().setVelocityConversionFactor(distancePerPulse);
    }

    @Override
    public double getOutputCurrent() {
        return ((CANSparkMax)controller).getOutputCurrent();
    }

    @Override
    public double getOutputVoltage() {
        return ((CANSparkMax)controller).getAppliedOutput() * ((CANSparkMax)controller).getBusVoltage();
    }

    @Override
    public void setIdleMode(frc.robot.infrastructure.IdleMode mode){
        switch(mode) {
            case kBrake:
                ((CANSparkMax)controller).setIdleMode(IdleMode.kBrake);
                break;
            case kCoast:
                ((CANSparkMax)controller).setIdleMode(IdleMode.kCoast);
                break;
        }

    }

    @Override
    public void setControlMode(SpeedControllerMode mode) {
        switch(mode) {
            case kIdle:
                break;
            case kDutyCycle:
                this.mode = ControlType.kDutyCycle;
                pidSlot = 0;
                break;
            case kVoltage:
                this.mode = ControlType.kVoltage;
                pidSlot = 0;
                break;
            case kCurrent:
                break;
            case kPosition:
                this.mode = ControlType.kPosition;
                pidSlot = 0;
                break;
            case kVelocity:
                this.mode = ControlType.kVelocity;
                pidSlot = Constants.kVelocitySlot;
                break;
            case kMotionMagic:
                break;
            case kMotionProfile:
                break;
        }
    }

    @Override
    public void setOpenLoopRampTime(double rate) {
        ((CANSparkMax)controller).setOpenLoopRampRate(rate);
    }

    @Override
    public void setClosedLoopRampTime(double time) {
        ((CANSparkMax)controller).setClosedLoopRampRate(time);
    }

    @Override
    public boolean getReverseLimit() {
        return false;
    }

    
}