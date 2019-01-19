package frc.robot.infrastructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class SparkMax extends SmartSpeedController {

    ControlType mode;

    public SparkMax(CANSparkMax spark) {
        super.controller = spark;
    }

    @Override
    public void set(double speed, SpeedControllerMode mode) {
        setControlMode(mode);
        ((CANSparkMax)controller).getPIDController().setReference(speed, this.mode);
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
        return ((CANSparkMax)controller).getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return ((CANSparkMax)controller).getEncoder().getVelocity();
    }

    @Override
    public double getOutputCurrent() {
        return ((CANSparkMax)controller).getOutputCurrent();
    }

    @Override
    public double getOutputVoltage() {
        return 0;
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
                break;
            case kVoltage:
                this.mode = ControlType.kVoltage;
                break;
            case kPosition:
                this.mode = ControlType.kPosition;
                break;
            case kVelocity:
                this.mode = ControlType.kVelocity;
                break;
            case kMotionMagic:
                break;
            case kMotionProfile:
                break;
        }
    }

}