package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Drivetrain {
    protected SensorTransmission leftDrive, rightDrive;
    protected DoubleSolenoid gearShifter;

    public Drivetrain(SensorTransmission left, SensorTransmission right, DoubleSolenoid shift) {
        leftDrive = left;
        rightDrive = right;
        gearShifter = shift;
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