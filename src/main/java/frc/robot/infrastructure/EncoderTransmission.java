package frc.robot.infrastructure;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.CustomPIDOutput;
import frc.robot.infrastructure.CustomEncoder;

public class EncoderTransmission extends Transmission {

    private CustomEncoder encoder;
    private PIDController pid;
    private CustomPIDOutput pidOutput;

    public EncoderTransmission(SmartSpeedController[] group, CustomEncoder enc, double pidPeriod) {
        super(group);
        encoder = enc;
        pidOutput = new CustomPIDOutput(); 
        pid = new PIDController(0, 0, 0, 0, encoder, pidOutput, pidPeriod);
    }

    public EncoderTransmission(SmartSpeedController[] group, CustomEncoder enc) {
        super(group);
        encoder = enc;
    }

    public double getPosition() {
        return encoder.getEncoderCounts();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
    
    public void reset() {
        encoder.zero();
    }
    public void setPIDSourceType(PIDSourceType type) {
        encoder.setPIDSourceType(type);
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        if(pid != null)
            pid.setPID(kP, kI, kD, kF);
        else
            System.out.println("Transmission has no PIDController object!");
    }

    public void setSetpoint(double setpoint) {
        if(pid != null)
            pid.setSetpoint(setpoint);
        else
            System.out.println("Transmission has no PIDController object!");
    }
    
    public double getPIDOutput() {
        return pidOutput.getOutput();
    }
    
    public void enablePID() {
        if(pid != null)
            pid.enable();
        else
            System.out.println("Transmission has no PIDController object!");
    }

    public void disablePID() {
        if(pid != null)
            pid.disable();
        else
            System.out.println("Transmission has no PIDController object!");
    }
}