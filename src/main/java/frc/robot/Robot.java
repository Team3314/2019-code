/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStateMachine;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CargoIntake.IntakeStateMachine;
import frc.robot.subsystems.Drive.DriveMode;

/**
 * The VM is configured to automatically run tHIs class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of tHIs class or the package after
 * creating tHIs project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static RobotMap map = new RobotMap();
  public static Drive drive = new Drive(map.leftDrive, map.rightDrive, map.navx);
  public static CargoIntake cargoIntake = new CargoIntake(map.intakeTransmission);
  public static HatchMechanism hatch = new HatchMechanism(map.gripperPiston, map.sliderPiston);
  public static Elevator elevator = new Elevator(map.elevatorTransmission);
  public static Camera camera = new Camera();
  public static Superstructure superstructure = new Superstructure();

  public static HumanInput HI = HumanInput.getInstance();

  /**
   * THIs function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
    superstructure.stopCompressor();
  }

  @Override
  public void autonomousPeriodic() { 

    allPeriodic();
    
  }

  @Override
  public void teleopInit() {
    drive.resetSensors();
    superstructure.stopCompressor();
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
  }

  @Override
  public void testInit() {
    superstructure.startCompressor();
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
    outputToSmartDashboard();

  }

}
