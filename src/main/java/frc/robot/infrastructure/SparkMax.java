package frc.robot.infrastructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class SparkMax extends SmartSpeedController {

    CANSparkMax spark;

    ControlType mode;

    public SparkMax(CANSparkMax spark) {
        this.spark = spark;
    }

    @Override
    public void set(double speed, SpeedControllerMode mode) {
        setControlMode(mode);
        spark.getPIDController().setReference(speed, this.mode);
    }

    @Override
    public void pidWrite(double output) {
        spark.set(output);
    }

    @Override
    public double getEncoderCounts() {
        return spark.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return spark.getEncoder().getVelocity();
    }

    @Override
    public double getOutputCurrent() {
        return spark.getOutputCurrent();
    }

    @Override
    public double getOutputVoltage() {
        return 0;
    }

    @Override
    public void setIdleMode(frc.robot.infrastructure.IdleMode mode){
        switch(mode) {
            case kBrake:
                spark.setIdleMode(IdleMode.kBrake);
                break;
            case kCoast:
                spark.setIdleMode(IdleMode.kCoast);
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