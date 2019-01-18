package frc.robot.infrastructure;

import frc.robot.infrastructure.CustomEncoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class SensorTransmission {

    SmartSpeedController[] motors;
    CustomEncoder encoder;

    public SensorTransmission(SpeedControllerGroup group, CustomEncoder enc) {
        motors = group;
        encoder = enc;
    }

    public void set(double input) {
        motors.set(input);
    }

    public void set(SpeedControllerMode mode, double input) {

    }

}