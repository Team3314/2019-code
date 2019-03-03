/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import frc.robot.autos.AutoTwoHatchRocketClose;
import frc.robot.autos.Autonomous;
import frc.robot.statemachines.GamePieceStateMachine;
import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorControlMode;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Camera.DSCamera;
import frc.robot.subsystems.CargoIntake.IntakeState;
import frc.robot.subsystems.Drive.DriveMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of tHIs class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static RobotMap map = new RobotMap();
  public static HumanInput HI = new HumanInput();
  public static Camera camera = new Camera(map.leftLightRing, map.rightLightRing);
  public static Drive drive = new Drive(map.leftDrive, map.rightDrive, map.navx, map.shifter, map.leftDriveEncoder, map.rightDriveEncoder, map.distanceSensor);
  public static Elevator elevator = new Elevator(map.elevatorTransmission);
  public static CargoIntake cargoIntake = new CargoIntake(map.intakeTransmission, map.outtakeTransmission, map.intakePiston);
  public static HatchMechanism hatch = new HatchMechanism(map.gripperPiston, map.sliderPiston);
  public static Superstructure superstructure = new Superstructure(map.compressor);
  public static Climber climber = new Climber(map.climberPiston, map.intakePiston, map.intakeToGroundPiston, map.highPressure);
  public static GamePieceStateMachine gamePieceStateMachine = new GamePieceStateMachine();

  public Autonomous autoMode = null;
  public AutoTwoHatchRocketClose auto = new AutoTwoHatchRocketClose();

  public Runnable smartDashboardRunnable = new Runnable(){
  
    @Override
    public void run() {
      outputToSmartDashboard();
    }
  };
  public Notifier smartDashboardNotifier = new Notifier(smartDashboardRunnable);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    smartDashboardNotifier.startPeriodic(.1);
    drive.resetSensors();
    elevator.resetSensors();
  }

  @Override
  public void disabledInit() {
    drive.resetDriveEncoders();
    cargoIntake.stopLoadingBall();
  }

  @Override
  public void disabledPeriodic() {
    allPeriodic();
  }
  @Override
  public void robotPeriodic() {
    HI.setHasGamepad(HI.getHasGamepad());

    if(HI.getForwardCamera())
      camera.setDSView(DSCamera.FRONT);
    else if(HI.getRightCamera())
      camera.setDSView(DSCamera.RIGHT);
    else if(HI.getBackCamera()) 
      camera.setDSView(DSCamera.BACK);
    else if(HI.getLeftCamera()) 
      camera.setDSView(DSCamera.LEFT);
  }
  @Override
  public void autonomousInit() {
    camera.setLightRings(true);
    drive.resetSensors();
    superstructure.stopCompressor();
    autoMode = AutoModeSelector.getSelectedAutoMode();
  }

  @Override
  public void autonomousPeriodic() { 
    if(!elevator.getHomed())
      elevator.setElevatorState(ElevatorControlMode.HOMING);
    if(autoMode == null) {
      teleopPeriodic();
    } 
    else {
      allPeriodic();
      autoMode.update();
    }
    
  }

  @Override
  public void teleopInit() {
    superstructure.startCompressor();
    if(elevator.getElevatorState() == ElevatorControlMode.MOTION_MAGIC) {
      elevator.set(elevator.getPosition());
    }
    else {
      elevator.set(0);
    }
  }


  @Override
  public void teleopPeriodic() {
    allPeriodic();

    drive.setVelocityControl(HI.getVelocityControl());
    if(HI.getAuto()) {
      auto.update();
    } else{
      auto.reset();
    if(cargoIntake.getCargoInCarriage()) {
      if(HI.getStoreElevatorLevel1()) {
        gamePieceStateMachine.setMode(GamePieceStateMachineMode.BALL_LEVEL1);
      }
      else if(HI.getStoreElevatorLevel2()) {
        gamePieceStateMachine.setMode(GamePieceStateMachineMode.BALL_LEVEL2);
      }
      else if(HI.getStoreElevatorLevel3()) {
        gamePieceStateMachine.setMode(GamePieceStateMachineMode.BALL_LEVEL3);
      }
    }
    else {
      if(HI.getStoreElevatorLevel1()) {
        gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_LEVEL1);
      }
      else if(HI.getStoreElevatorLevel2()) {
        gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_LEVEL2);
      }
      else if(HI.getStoreElevatorLevel3()) {
        gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_LEVEL3);
      }
    }
    if(HI.getStoreElevatorPickup()) {
      gamePieceStateMachine.setMode(GamePieceStateMachineMode.HATCH_PICKUP);
    }
    if(HI.getAutoGamePiece()) {
      gamePieceStateMachine.setRequest(true);
      if(HI.getGyrolock())
        drive.setTank(HI.getLeftThrottle(),HI.getLeftThrottle(), 2);
    }
    else {
      gamePieceStateMachine.setRequest(false);
      if(HI.getClimbMode()) {
        climber.setClimbRequest(true);
      }
      else if(HI.getAbortClimb()) {
        climber.setClimbRequest(false);
      }
      if(HI.getResetGyro()) {
        drive.resetSensors();
      }
      // Drive Controls
      if(HI.getGyrolock()) {
        drive.setDriveMode(DriveMode.GYROLOCK);
        drive.setTank(HI.getLeftThrottle(), HI.getLeftThrottle(), 2);
        if(HI.turnToZero())
          drive.setDesiredAngle(0);
        else if(HI.turnToRight())
          drive.setDesiredAngle((-90));
        else if(HI.turnBack())
          drive.setDesiredAngle(180);
        else if(HI.turnToLeft())
          drive.setDesiredAngle(90);
      }
      else if(HI.getVision()) {
        drive.setDriveMode(DriveMode.VISION_CONTROL);
        drive.setTank(HI.getRightThrottle(), HI.getRightThrottle(), 2);
      }
      else {  
        drive.setDriveMode(DriveMode.TANK);
        drive.setTank(HI.getLeftThrottle(), HI.getRightThrottle(), 2);
      }
      if(HI.getHighGear()) {
        drive.setHighGear(true);
      }
      else if(HI.getLowGear()) {
        drive.setHighGear(false);
      }
      if(HI.getLightRingsOn())
        camera.setLightRings(true);
      else if(HI.getLightRingsOff()) 
        camera.setLightRings(false);
      drive.setElevatorUp(elevator.getPosition() >= Constants.kElevatorLowAccelerationThreshold);

      /**
       * ELEVATOR CONTROLS
      */
      if(HI.getCargoEject()) {
        cargoIntake.stopLoadingBall();
      }
      hatch.setPlaceRequest(HI.getAutoHatchPlace());
      hatch.setRetractRequest(HI.getAutoHatchRetract());
      if(!elevator.isHoming()) {
        if(!HI.getElevatorManual()) {
          cargoIntake.setIntakeRequest(HI.getAutoCargoIntake()); 
          hatch.setIntakeRequest(HI.getAutoHatchIntake());
        }
        if(!cargoIntake.getLoadingBall() && !HI.getAutoHatchIntake() && !HI.getAutoHatchPlace()) {
          if(HI.getElevatorManual()) {
            elevator.setElevatorState(ElevatorControlMode.MANUAL);
            elevator.set(HI.getElevatorSpeed());
          }
          else if(HI.getHome()) {
            elevator.setElevatorState(ElevatorControlMode.HOMING);
          }
          else {
            elevator.setElevatorState(ElevatorControlMode.MOTION_MAGIC);
            if(HI.getElevatorLevel1()) {
              if(cargoIntake.getCargoInCarriage()) {
                elevator.set(Constants.kElevatorBallLevel1);
              }
              else {
                elevator.set(Constants.kElevatorHatchLevel1);
              }
            }
            else if (HI.getElevatorLevel2()) {
              if(cargoIntake.getCargoInCarriage()) {
                elevator.set(Constants.kElevatorBallLevel2);
              }
              else {
                elevator.set(Constants.kElevatorHatchLevel2);
              }
            }
            else if(HI.getElevatorLevel3()) {
              if(cargoIntake.getCargoInCarriage()) {
                elevator.set(Constants.kElevatorBallLevel3);
              }
              else {
                elevator.set(Constants.kElevatorHatchLevel3);
              }
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
          /**
           * CLIMBER CONTROLS
           */
      }
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
    camera.update();
    if(HI.getLightRingsOn())
      camera.setLightRings(true);
    if(HI.getLightRingsOff())
      camera.setLightRings(false);
  }

  public void outputToSmartDashboard() {
	  cargoIntake.outputToSmartDashboard();
		drive.outputToSmartDashboard();
    hatch.outputToSmartDashboard();
    elevator.outputToSmartDashboard();
		camera.outputToSmartDashboard();
    superstructure.outputToSmartDashboard();
    climber.outputToSmartDashboard();
    gamePieceStateMachine.outputToSmartDashboard();
  }

  public void allPeriodic() {
    drive.update();
    camera.update();
    cargoIntake.update();
    elevator.update();  
    superstructure.update();
    hatch.update();
    climber.update();
    gamePieceStateMachine.update();
  }

}