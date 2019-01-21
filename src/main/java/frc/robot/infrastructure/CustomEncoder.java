package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.PIDSource;

public interface CustomEncoder extends PIDSource {

    /*Required Functionality:
        Get position
        get velocity
    */
    public double getEncoderCounts();

    public double getVelocity();

    public void zero();

    public void setDistancePerPulse(double distancePerPulse);

    public double getDistance();
}