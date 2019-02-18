package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;

public class Climber implements Subsystem {

    private DoubleSolenoid climberPiston;
    private boolean climberDown;

    public Climber(DoubleSolenoid climberPiston) {
        this.climberPiston = climberPiston;
    }

    @Override
    public void update() {
        if(climberDown)
            climberPiston.set(Constants.kClimberDown);
        else
            climberPiston.set(Constants.kClimberUp);
    }

    @Override
    public void outputToSmartDashboard() {

    }

    public void setClimberDown(boolean down) {
        climberDown = down;
    }

    @Override
    public void resetSensors() {

    }

    

    

}