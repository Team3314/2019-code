package frc.robot.infrastructure;

public interface CustomEncoder {

    /*Required Functionality:
        Get position
        get velocity
    */
    public double getEncoderCounts();

    public double getVelocity();

    public void zero();
}