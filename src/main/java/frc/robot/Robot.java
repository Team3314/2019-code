/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.SparkDrive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SparkDrive.driveMode;

/**
 * The VM is configured to automatically run tHIs class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of tHIs class or the package after
 * creating tHIs project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SparkDrive drive = SparkDrive.getInstance();
  private Intake intake = Intake.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private Camera camera = Camera.getInstance();
  private Superstructure superStructure = Superstructure.getInstance();

  private HumanInput HI = HumanInput.getInstance();

  boolean lastGyrolock = false;

  /**
   * THIs function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
    superStructure.stopCompressor();
  }

  @Override
  public void autonomousPeriodic() { 

    allPeriodic();
    
  }

  @Override
  public void teleopInit() {
    drive.resetSensors();
    superStructure.stopCompressor();
  }

  @Override
  public void teleopPeriodic() {

    allPeriodic();

    // Drive Controls
		if(HI.getGyrolock()) {
			if(!lastGyrolock) {
				drive.setDriveMode(driveMode.GYROLOCK);
			}
			drive.setDesiredSpeed(HI.getLeftThrottle());
		}
		
		if(HI.getHighGear()) {
			drive.setHighGear(true);
		}
		else if(HI.getLowGear()) {
      drive.setHighGear(false);
    }
    drive.setStickInputs(HI.getLeftThrottle(), HI.getRightThrottle());

    lastGyrolock = HI.getGyrolock();
  }

  @Override
  public void testInit() {
    superStructure.startCompressor();
  }

  @Override
  public void testPeriodic() {
  }

  public void allPeriodic() {

    camera.update();
    drive.update();
    intake.update();
    elevator.update();
    superStructure.update();

  }

}
