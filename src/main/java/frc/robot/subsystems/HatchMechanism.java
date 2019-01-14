package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;

public class HatchMechanism implements Subsystem {

    private static HatchMechanism mInstance = new HatchMechanism();

    public static HatchMechanism getInstance() {
        return mInstance;
    }

    private DoubleSolenoid hatchPiston;

    @Override
    public void update() {

    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void resetSensors() {

    }

}