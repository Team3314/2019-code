package frc.robot.infrastructure;

public class Lift {
    SensorTransmission transmission;

    public Lift(SensorTransmission t) {
        transmission = t;
    }

    public void set(double command) {
        transmission.set(command);
    }
}