package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.PIDController;
import frc.robot.infrastructure.CustomEncoder;

public class SensorTransmission extends Transmission {

    private CustomEncoder encoder;
    private PIDController pid;

    public SensorTransmission(SmartSpeedController[] group, CustomEncoder enc) {
        super(group);
        encoder = enc;
        //pid = new PIDController(Kp, Ki, Kd, Kf, encoder, motors[0]);
        setName("SensorTransmission", Transmission.instances);
    }

    public double getPosition() {
        return encoder.getEncoderCounts();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
    
    public void reset() {
        encoder.zero();
    }

}