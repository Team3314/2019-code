package frc.robot.infrastructure;

import frc.robot.infrastructure.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class SensorTransmission {

    SpeedControllerGroup motors;
    Encoder encoder;

    public SensorTransmission(SpeedControllerGroup group, Encoder enc) {
        motors = group;
        encoder = enc;
    }

    public void set(double input) {
        motors.set(input);
    }

}