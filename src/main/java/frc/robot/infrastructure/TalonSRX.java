package frc.robot.infrastructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonSRX extends SmartSpeedController {

    WPI_TalonSRX talon;

    ControlMode mode;

    public TalonSRX(WPI_TalonSRX talon) {
        this.talon = talon;
    }
    @Override
    public void set(double speed, SpeedControllerMode mode) {
        setControlMode(mode);
        talon.set(this.mode,speed);
    }

    @Override
    public void pidWrite(double output) {
        talon.set(output);
    }

    @Override
    public double getEncoderCounts() {
        return talon.getSelectedSensorPosition();
    }

    @Override
    public double getVelocity() {
        return talon.getSelectedSensorVelocity();
    }

    @Override
    public double getOutputCurrent() {
        return talon.getOutputCurrent();
    }

    @Override
    public double getOutputVoltage() {
        return talon.getMotorOutputVoltage();
    }

    @Override
    public void setIdleMode(frc.robot.infrastructure.IdleMode mode){
        switch(mode) {
            case kBrake:
                talon.setNeutralMode(NeutralMode.Brake);
                break;
            case kCoast:
                talon.setNeutralMode(NeutralMode.Coast);
                break;
        }

    }
    @Override
    public void setControlMode(SpeedControllerMode mode) {
        switch(mode) {
            case kIdle:
                break;
            case kDuty_Cycle:
                this.mode = ControlMode.PercentOutput;
                break;
            case kVoltage:
                break;
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
    

}