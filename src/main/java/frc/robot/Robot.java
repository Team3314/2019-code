/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.swing.text.StyledEditorKit.AlignmentAction;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Drive;
import frc.robot.autos.Autonomous;
import frc.robot.autos.DoubleHatchAuto;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStateMachine;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CargoIntake.IntakeStateMachine;
import frc.robot.subsystems.Drive.DriveMode;
import frc.robot.AlignmentStateMachine;

/**
 * The VM is configured to automatically run tHIs class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of tHIs class or the package after
 * creating tHIs project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static RobotMap map = new RobotMap();
  public static Camera camera = new Camera();
  public static Drive drive = new Drive(map.leftDrive, map.rightDrive, map.navx, map.shifter);
  public static CargoIntake cargoIntake = new CargoIntake(map.intakeTransmission);
  public static HatchMechanism hatch = new HatchMechanism(map.gripperPiston, map.sliderPiston);
  public static Elevator elevator = new Elevator(map.elevatorTransmission);
  public static Superstructure superstructure = new Superstructure(map.compressor);
  DoubleHatchAuto auto1 = new DoubleHatchAuto();
  AlignmentStateMachine aligning = new AlignmentStateMachine();
  public Runnable smartDashboardRunnable = new Runnable(){
  Autonomous selectedAutoMode = null;


    @Override
    public void run() {
      outputToSmartDashboard();
    }
  };

  public Notifier smartDashboardNotifier = new Notifier(smartDashboardRunnable);

  public static HumanInput HI = HumanInput.getInstance();

  /**
   * THIs function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    smartDashboardNotifier.startPeriodic(.1);
    drive.resetSensors();
    auto1.reset();
  }

  @Override
  public void disabledInit() {
    drive.resetDriveEncoders();
  }

  @Override
  public void disabledPeriodic() {
    
  }
  @Override
  public void robotPeriodic() {
    outputToSmartDashboard();
}
  @Override
  public void autonomousInit() {
    drive.resetSensors();
    auto1.reset();
    selectedAutoMode = selector.getSelectedAutoMode();
  }

  @Override
  public void autonomousPeriodic() { 
    auto1.update();
    allPeriodic();
  }

  @Override
  public void teleopInit() {
    drive.resetSensors();
    aligning.reset();
  }

  @Override
  public void teleopPeriodic() {

    allPeriodic();
    // Drive Controls
		if(HI.getGyrolock()) {
      drive.setDriveMode(DriveMode.GYROLOCK);
      drive.set(HI.getLeftThrottle(), HI.getLeftThrottle());
    }
    else if (HI.getVision()) {
      drive.setDriveMode(DriveMode.VISION_CONTROL);
    }
    else if (HI.getVelocityControl()) {
      drive.setDriveMode(DriveMode.VELOCITY);
      drive.set(HI.getLeftThrottle(), HI.getRightThrottle());
    }
    else {  
      drive.setDriveMode(DriveMode.OPEN_LOOP);
      drive.set(HI.getLeftThrottle(), HI.getRightThrottle());
    }

		if(HI.getHighGear()) {
			drive.setHighGear(true);
		}
		else if(HI.getLowGear()) {
      drive.setHighGear(false);
    }

    /**
     * ELEVATOR CONTROLS
    */
    if(HI.getElevatorManual()) { 
      elevator.setElevatorState(ElevatorStateMachine.MANUAL);
      elevator.set(HI.getElevatorSpeed());
    }
    else {
      elevator.setElevatorState(ElevatorStateMachine.MOTION_MAGIC);
    }

    /**
     * CARGO INTAKE CONTROLS
    */
    if (HI.getCargoIntake()) {
      cargoIntake.setIntakeState(IntakeStateMachine.INTAKING);
    }
    else if (HI.getCargoRelease()) {
      cargoIntake.setIntakeState(IntakeStateMachine.RELEASING);
    }
    else if (!HI.getCargoIntake() && !HI.getCargoRelease()) {
      cargoIntake.setIntakeState(IntakeStateMachine.HOLDING);
    }
    /**
     * HATCH MECH CONTROLS
    */

    if (HI.getGripperDown()) {
      hatch.setGripperDown(true);
    }
    else if (HI.getGripperUp()) {
      hatch.setGripperDown(false);
    }
    if (HI.getSliderOut()) {
      hatch.setSliderOut(true);
    }
    else if (HI.getSliderIn()) {
      hatch.setSliderOut(false);
    }

    /**
     * TRIGGER ALIGNMENT
    */
    if (HI.getVision()) { 
      aligning.update();
    }


  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void outputToSmartDashboard() {
	  cargoIntake.outputToSmartDashboard();
		drive.outputToSmartDashboard();
    hatch.outputToSmartDashboard();
    elevator.outputToSmartDashboard();
		camera.outputToSmartDashboard();
		superstructure.outputToSmartDashboard();
  }

  public void allPeriodic() {
    drive.update();
    camera.update();
    cargoIntake.update();
    elevator.update();  
    superstructure.update();
    hatch.update();
    
  }

}
