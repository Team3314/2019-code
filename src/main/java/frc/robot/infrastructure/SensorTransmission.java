package frc.robot.infrastructure;

import frc.robot.infrastructure.CustomEncoder;

public class SensorTransmission extends Transmission {

    private CustomEncoder encoder;

    public SensorTransmission(SmartSpeedController[] group, CustomEncoder enc) {
        super(group);
        encoder = enc;
        setName("SensorTransmission", super.instances);
    }

    public double getPosition() {
        return encoder.getEncoderCounts();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

}