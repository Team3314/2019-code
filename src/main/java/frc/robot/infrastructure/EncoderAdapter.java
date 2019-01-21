package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderAdapter implements CustomEncoder {

    private Encoder encoder;
    private PIDSourceType pidSource;

    public EncoderAdapter(Encoder enc) {
        encoder = enc;
    }

    @Override
    public double getEncoderCounts() {
        return encoder.get();
    }

    @Override
    public double getVelocity() {
        return encoder.getRate(); 
    }

    @Override
    public void zero() {
        encoder.reset();
    }
    
    @Override
    public void setDistancePerPulse(double distancePerPulse){ 
        encoder.setDistancePerPulse(distancePerPulse);
    }

    @Override
    public double getDistance() {
        return encoder.getDistance();
    } 
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSource = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSource;
    }

    @Override
    public double pidGet() {
        if(pidSource == PIDSourceType.kDisplacement)
            return getDistance();
        else if(pidSource == PIDSourceType.kRate)
            return getVelocity();
        else
            return -1;
	}

}