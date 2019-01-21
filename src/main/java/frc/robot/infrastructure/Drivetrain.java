package frc.robot.infrastructure;

public class Drivetrain {
    protected EncoderTransmission leftDrive, rightDrive;
    protected double leftDemand, rightDemand;

    public Drivetrain(EncoderTransmission left, EncoderTransmission right) {
        leftDrive = left;
        rightDrive = right;
    }

    public void setLeft(double input) {
        leftDrive.set(input);
    }

    public void setRight(double input) {
        rightDrive.set(input);
    }

    public void set(double left, double right) {
        leftDemand = left;
        rightDemand = right;
    }
}