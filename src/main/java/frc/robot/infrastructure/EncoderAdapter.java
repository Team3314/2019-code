package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderAdapter implements CustomEncoder {

    private Encoder encoder;

    public EncoderAdapter(Encoder enc) {
        encoder = enc;
    }

    @Override
    public int getEncoderCounts() {
        return encoder.get();
    }

    @Override
    public double getVelocity() {
        return encoder.getRate(); 
    }

}