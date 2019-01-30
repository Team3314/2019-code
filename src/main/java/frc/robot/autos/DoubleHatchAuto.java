/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be acconied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.infrastructure.SparkMax;

/**
 * Add your docs here.
 */
/*Changes for final code 
-Encoder ticks to inches
-Placing hatch simulation replacement
*/
public class DoubleHatchAuto extends Autonomous{

  public enum states {
    START,
    START_FORWARD,                    // Go forward off the platform 48 inches
    ROTATE1_TO_ROCKET1,               // Blind rotate to gyro heading of ~-30
    ALIGN1,                           // Align robot to center of rocket target
    FORWARD_TO_TARGET1,               // Go towards rocket while aligning. Stop when collision.
    PLACE_HATCH1,                     // Simulate placing hatch wait for a second
    BACKUP_AFTER_PLACING_HATCH,       // Go backwards 24-36 inches?
    TURN_AROUND,                      // Turn to a heading of -180
    DRIVE_TO_LOADING_STATION,         // Drive and align towards loading station until collision
    RETREIVE_HATCH2,                  // Simulate picking up hatch panel and wait a second
    BACK_FROM_LOADING_STATION,        // Drive backwards at a heading of -175 for ~23 feet
    ROTATE2_TO_ROCKET,                // Blind rotate to gyro heading of -150
    ALIGN2,                           // Align robot to center of target
    FORWARD_TO_TARGET2,               // Go towards rocket while aligning until collision.
    STOP
  }

  states currentState;
  WPI_TalonSRX leftMasterTalon, rightMasterTalon;
  int cyclesInState;
  double baseSpeed = 0.5; 
  double inchesTraveledInState;
  String waitingFor = "";

  public DoubleHatchAuto() {
    currentState = states.START;
  }
  
  @Override
  public void reset() {
    currentState = states.START;
  }
  
  @Override
  public void update() {
    switch (currentState) {
      case START:
      resetDriveEncoders();
      currentState = states.ALIGN1;
      //currentState = states.START_FORWARD;
      break;

      case START_FORWARD:
        drivePower(0.5);
        if(getAveragePosition() >= 48){
          currentState = states.ROTATE1_TO_ROCKET1;
          resetDriveEncoders();
          driveGyrolock(0, -30);
        }
        waitingFor = "Go forward 48 Inches";
      break;

      case ROTATE1_TO_ROCKET1:
        if(gyroTurnDone()){
          currentState = states.ALIGN1;
        }
        waitingFor = "Put gyro heading to -30";
      break;

      case ALIGN1:
        drivePower(getCorrection(), -(getCorrection()));
        if(Math.abs(getCorrection()) <= 0.07 && isTargetInView()){
          currentState = states.ALIGN1;
          //currentState = states.FORWARD_TO_TARGET1;
        }
        waitingFor = "Target to Center";
      break;

    case FORWARD_TO_TARGET1:    
        drivePower(0.5 - getCorrection(), 0.5 + getCorrection());
        if(collision() == true){
          currentState = states.PLACE_HATCH1;
          startTimer();
        }
        waitingFor = "Collision";
      break;

    case PLACE_HATCH1:
      drivePower(0);
      if(getTime() >= 1){
        resetTimer();
        currentState = states.BACKUP_AFTER_PLACING_HATCH;
        resetDriveEncoders();
      }
      waitingFor = "Waiting 1 second";
      break;

    case BACKUP_AFTER_PLACING_HATCH:
      drivePower(-0.5);
      if (getAveragePosition() <= -30){
        currentState = states.TURN_AROUND;
        driveGyrolock(0, -180);
        //resetDriveEncoders();
      }
      waitingFor = "Back up 30 inches";
      break;

    case TURN_AROUND:
      if(gyroTurnDone()){
        currentState = states.DRIVE_TO_LOADING_STATION;
      }
        waitingFor = "Put gyro heading to -180"; 
      break;

    case DRIVE_TO_LOADING_STATION:
      drivePower(0.5 - getCorrection(), 0.5 + getCorrection());
      if(collision() == true){
        currentState = states.RETREIVE_HATCH2;
        startTimer();
      }
      waitingFor = "Collision";
      break;

    case RETREIVE_HATCH2:
      drivePower(0);
      if(getTime() >= 1) {
        resetTimer();
        currentState = states.BACK_FROM_LOADING_STATION;
        resetDriveEncoders();
      }
      waitingFor = "Waiting 1 second";
      break;

    case BACK_FROM_LOADING_STATION:
     drivePower(-0.5);
     if (getAveragePosition() <= -276){
        currentState = states.ROTATE2_TO_ROCKET;
        resetDriveEncoders();
      driveGyrolock(0, -150);
     }
      waitingFor = "Go back 276 inches";
      break;

    case ROTATE2_TO_ROCKET: 
      if(gyroTurnDone()){
        currentState = states.ALIGN2;
      }
      waitingFor = "Put gyro heading to -150";
      break;

    case ALIGN2:
      drivePower(getCorrection(), -getCorrection());
      if(Math.abs(getCorrection()) <= 0.07 && targetInView()){
        currentState = states.FORWARD_TO_TARGET2;
      }
      waitingFor = "Align to center of target";
      break;

    case FORWARD_TO_TARGET2:
      drivePower(0.5 - getCorrection(), 0.5 + getCorrection());
      if(collision()){
        currentState = states.STOP;
      }
      waitingFor = "Collision";
      break;

    case STOP:
      drivePower(0);
      waitingFor = "We stopped";
      break;
    }
    SmartDashboard.putString("Waiting For", waitingFor);
    SmartDashboard.putString("State", currentState.toString());
    SmartDashboard.putNumber("Avg. Position", getAveragePosition());
    SmartDashboard.putBoolean("Gyro Turn Done", gyroTurnDone());
    SmartDashboard.putNumber("Accelerometer", getAccelerometer());
    SmartDashboard.putBoolean("collision?", collision());
    SmartDashboard.putNumber("Correction", getCorrection());
    "hello"
  }
}