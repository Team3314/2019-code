package frc.robot.infrastructure;

public class Drivetrain {
    protected EncoderTransmission leftDrive, rightDrive;
    protected double leftDemand, rightDemand;

    public Drivetrain(EncoderTransmission left, EncoderTransmission right) {
        leftDrive = left;
        rightDrive = right;
    }

    public void update() {
        leftDrive.set(leftDemand);
        rightDrive.set(rightDemand);
    }

    public void setLeft(double input) {
        leftDemand = input;
    }

    public void setRight(double input) {
        rightDemand = input;
    }

    public void set(double left, double right) {
        leftDemand = left;
        rightDemand = right;
    }

    public void setTank(double left, double right, double power) {
        leftDemand = Math.copySign(Math.pow(left, power), left);
        rightDemand = Math.copySign(Math.pow(right, power), right);
    }
/*
    public void setTank(double left, double right, double power, double turningSensitivity) {
        double averageInput = (left + right) / 2;
        double differential = (left - right) / 2 * turningSensitivity;
        left = averageInput + differential;
        right = averageInput - differential;
        leftDemand = Math.copySign(Math.pow(left, power), left);
        rightDemand = Math.copySign(Math.pow(right, power), right);
    }*/
    public void setTank(double left, double right, double power, double turningSensitivity) {
        if((left > 0 && right > 0) || (left <0 && right < 0)) {
            double translation = Math.copySign(Math.min(Math.abs(left), Math.abs(right)), left);
            left -= translation;
            right -= translation;
            left *= turningSensitivity;
            right *= turningSensitivity;
            left += translation;
            right += translation;
        } else {
            left *= turningSensitivity;
            right *= turningSensitivity;
        }
        leftDemand = left;
        rightDemand = right;
    }
    public void setArcade(double speed, double turn, double power) {

    }
}