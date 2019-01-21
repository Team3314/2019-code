package frc.robot.infrastructure;

public class Lift {
    protected EncoderTransmission transmission;
    protected double demand;

    public Lift(EncoderTransmission t) {
        transmission = t;
    }

    public void set(double command) {
        demand = command;
    }
}