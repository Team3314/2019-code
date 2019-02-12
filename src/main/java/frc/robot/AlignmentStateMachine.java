package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class AlignmentStateMachine{
    states nextState, currentState;
    String waitingFor;

private Camera camera;{
    camera = Robot.camera;
}

private Drive drive;{
    drive = Robot.drive;
}

public AlignmentStateMachine(){
    currentState = states.START;
}

private Timer timer = new Timer();

public enum states{ 
    START,
    ALIGN, // Align robot to center of rocket target
    FORWARD_TO_TARGET, // Go towards rocket while aligning. Stop when collision.
    PLACE_HATCH, // Simulate placing hatch wait for a second
    STOP // Stop
  }

  public void reset(){
      currentState = states.START;
      SmartDashboard.putString("State", currentState.toString());
  }
  
  public void update(){
  switch(currentState){
    case START:
    currentState = states.ALIGN;

    case ALIGN:
    drive.set(camera.getCorrection(), - (camera.getCorrection()));
    if (Math.abs(camera.getCorrection()) <= 0.07 && camera.isTargetInView()) {
      //currentState = states.STOP;
      currentState = states.FORWARD_TO_TARGET;

    }
    waitingFor = "Target to Center";
    break;

  case FORWARD_TO_TARGET:
    drive.set(0.5 + camera.getCorrection(), 0.5 - camera.getCorrection());
    if (drive.collision()) {
      currentState  = states.PLACE_HATCH;
      timer.start();
    }
    String waitingFor = "Collision";
    break;

  case PLACE_HATCH:
    drive.set(0, 0);
    if (timer.get() >= 1) {
      timer.stop(); 
      timer.reset();
      currentState = states.STOP;
      drive.resetDriveEncoders();
    }
    waitingFor = "Waiting 1 second";
    break;

  case STOP:
    drive.set(0, 0);
    waitingFor = "We stopped";
    break;
  }
SmartDashboard.putString("State", currentState.toString());
SmartDashboard.putString("Waiting for", waitingFor);
}


}