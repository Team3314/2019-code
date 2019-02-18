package frc.robot.infrastructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonSRX extends SmartSpeedController {

    ControlMode mode;
    int pidSlot;

    public TalonSRX(WPI_TalonSRX talon) {
        super.controller = talon;
    }
    @Override
    public void set(double speed, SpeedControllerMode mode) {
        setControlMode(mode);
        if(isInverted)
            ((WPI_TalonSRX)controller).set(this.mode,-speed);
        else
            ((WPI_TalonSRX)controller).set(this.mode,speed);
    }

    @Override
    public void pidWrite(double output) {
        ((WPI_TalonSRX)controller).set(output);
    }

    @Override
    public double getEncoderCounts() {
        return ((WPI_TalonSRX)controller).getSelectedSensorPosition();
    }

    @Override
    public double getVelocity() {
        return ((WPI_TalonSRX)controller).getSelectedSensorVelocity();
    }

    @Override
    public void zero() {
        ((WPI_TalonSRX)controller).setSelectedSensorPosition(0,0,0);
    }

    @Override
    public double getOutputCurrent() {
        return ((WPI_TalonSRX)controller).getOutputCurrent();
    }

    @Override
    public double getOutputVoltage() {
        return ((WPI_TalonSRX)controller).getMotorOutputVoltage();
    }

    @Override
    public void setIdleMode(frc.robot.infrastructure.IdleMode mode){
        switch(mode) {
            case kBrake:
                ((WPI_TalonSRX)controller).setNeutralMode(NeutralMode.Brake);
                break;
            case kCoast:
                ((WPI_TalonSRX)controller).setNeutralMode(NeutralMode.Coast);
                break;
        }

    }
    @Override
    public void setControlMode(SpeedControllerMode mode) {
        switch(mode) {
            case kIdle:
                this.mode = ControlMode.Disabled;
                break;
            case kDutyCycle:
                this.mode = ControlMode.PercentOutput;
                break;
            case kVoltage:
                break;
            case kCurrent:
                this.mode = ControlMode.Current;
            case kPosition:
                this.mode = ControlMode.Position;
                break;
            case kVelocity:
                this.mode = ControlMode.Velocity;
                break;
            case kMotionMagic:
                this.mode = ControlMode.MotionMagic;
                break;
            case kMotionProfile:
                this.mode = ControlMode.MotionProfile;
                break;
        }
    }

    @Override
    public void setOpenLoopRampRate(double rate) {
        ((WPI_TalonSRX)controller).configOpenloopRamp(rate);
    }

    public boolean getReverseLimit() {
        return ((WPI_TalonSRX)controller).getSensorCollection().isRevLimitSwitchClosed();
    }
    

}