package frc.robot.infrastructure;

import frc.robot.infrastructure.CustomEncoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class SensorTransmission {

    SmartSpeedControllerGroup[] motors;
    CustomEncoder encoder;

    public SensorTransmission(SmartSpeedControllerGroup[] group, CustomEncoder enc) {
        motors = group;
        encoder = enc;
    }

    public SensorTransmission(SmartSpeedControllerGroup[] group) {
        motors = group;
    }

    public void set(double input) {
        motors.set(input);
    }

    public void set(SpeedControllerMode mode, double input) {

    }

}