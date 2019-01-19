package frc.robot.infrastructure;

public class Drivetrain {
    protected SensorTransmission leftDrive, rightDrive;

    public Drivetrain(SensorTransmission left, SensorTransmission right) {
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
        leftDrive.set(left);
        rightDrive.set(right);
    }
}