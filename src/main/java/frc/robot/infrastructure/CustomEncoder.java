package frc.robot.infrastructure;

public interface CustomEncoder {

    /*Required Functionality:
        Get position
        get velocity
    */
    public int getEncoderCounts();

    public double getVelocity();

}