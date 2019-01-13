package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class CargoIntake implements Subsystem {

    private static CargoIntake mInstance = new CargoIntake();
    
    private WPI_TalonSRX mIntakeMotor;

    public static CargoIntake getInstance() {
        return mInstance;
    }

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