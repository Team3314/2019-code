package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class SmartSpeedControllerGroup extends SpeedControllerGroup {

    public SmartSpeedControllerGroup() {
        super(SpeedController speedController,
              SpeedController... speedControllers);
    }

}