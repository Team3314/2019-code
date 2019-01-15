package frc.robot.infrastructure;

public class Drivetrain {
    private SensorTransmission leftDrive, rightDrive;

    public Drivetrain(SensorTransmission left, SensorTransmission right) {
        leftDrive = left;
        rightDrive = right;
    }
}