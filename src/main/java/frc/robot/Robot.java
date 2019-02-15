/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Drive;
import frc.robot.statemachines.CargoIntakeStateMachine;
import frc.robot.statemachines.HatchIntakeStateMachine;
import frc.robot.autos.DoubleHatchAuto;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorControlMode;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CargoIntake.IntakeState;
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
  public static Drive drive = new Drive(map.leftDrive, map.rightDrive, map.navx, map.shifter, map.leftDriveEncoder, map.rightDriveEncoder);
  public static CargoIntake cargoIntake = new CargoIntake(map.intakeTransmission, map.outtakeTransmission, map.pivotPiston);
  public static HatchMechanism hatch = new HatchMechanism(map.gripperPiston, map.sliderPiston);
  public static Elevator elevator = new Elevator(map.elevatorTransmission);
  public static Camera camera = new Camera();
  public static Superstructure superstructure = new Superstructure(map.compressor);
  public static HumanInput HI = new HumanInput();
  public static CargoIntakeStateMachine cargoIntakeStateMachine = new CargoIntakeStateMachine();
  public static HatchIntakeStateMachine hatchIntakeStateMachine = new HatchIntakeStateMachine();

  DoubleHatchAuto auto1 = new DoubleHatchAuto();
  public Runnable smartDashboardRunnable = new Runnable(){
  
    @Override
    public void run() {
      outputToSmartDashboard();
    }
  };
  public Notifier smartDashboardNotifier = new Notifier(smartDashboardRunnable);

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
    cargoIntake.outputToSmartDashboard();
  }
  @Override
  public void robotPeriodic() {
    //outputToSmartDashboard();
  }
  @Override
  public void autonomousInit() {
    drive.resetSensors();
    superstructure.stopCompressor();
    auto1.reset();
  }

  @Override
  public void autonomousPeriodic() { 
    auto1.update();
    allPeriodic();
    
  }

  @Override
  public void teleopInit() {
    superstructure.startCompressor();
  }

  @Override
  public void teleopPeriodic() {

    allPeriodic();
    //Switches between control of robot between action queue and manual 
    if(HI.getClearQueue()) {
      superstructure.clearQueue();
    }
    if(superstructure.isQueueDone()) {
      if(HI.getAutoGamePiece()) {
        superstructure.setAutoGamePiece(HI.getElevatorPlaceLevel());
      }
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

      drive.setElevatorUp(elevator.getPosition() >= Constants.kElevatorLowAccelerationThreshold);

      /**
       * ELEVATOR CONTROLS
      */
      if(HI.getAutoCargoIntake() && !HI.getElevatorManual()) {
        cargoIntakeStateMachine.setIntakeRequest(true);
      }
      else if(HI.getAutoHatchIntake() && !HI.getElevatorManual()) {
        hatchIntakeStateMachine.setIntakeRequest(true);
      }
      else {
        cargoIntakeStateMachine.setIntakeRequest(false);
        if(HI.getElevatorManual()) { 
          elevator.setElevatorState(ElevatorControlMode.MANUAL);
          elevator.set(HI.getElevatorSpeed());
        }
        else {
          elevator.setElevatorState(ElevatorControlMode.MOTION_MAGIC);
          if(HI.getElevatorLevel1()) {
            elevator.set(Constants.kElevatorLevel1);
          }
          else if (HI.getElevatorLevel2()) {
            elevator.set(Constants.kElevatorLevel2);
          }
          else if(HI.getElevatorLevel3()) {
            elevator.set(Constants.kElevatorLevel3);
          }
          
        }

      /**
       * CARGO INTAKE CONTROLS
      */
        if (HI.getCargoIntake()) {
          cargoIntake.setIntakeState(IntakeState.INTAKING);
        }
        else if (HI.getCargoPlace()) {
          cargoIntake.setIntakeState(IntakeState.PLACE);
        }
        else if (HI.getCargoTransfer()) {
          cargoIntake.setIntakeState(IntakeState.TRANSFERRING);
        }
        else if(HI.getCargoEject()) {
          cargoIntake.setIntakeState(IntakeState.VOMIT);
        }
        else {
          cargoIntake.setIntakeState(IntakeState.WAITING);
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
    cargoIntakeStateMachine.outputToSmartDashboard();
    hatchIntakeStateMachine.outputToSmartDashboard();
  }

  public void allPeriodic() {
    drive.update();
    camera.update();
    cargoIntake.update();
    elevator.update();  
    superstructure.update();
    hatch.update();
    cargoIntakeStateMachine.update();
    hatchIntakeStateMachine.update();

  }

}