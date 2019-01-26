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

/**
 * Add your docs here.
 */
/*Changes for final code 
-Encoder ticks to inches
-Placing hatch simulation replacement
*/
public class DoubleHatchAuto extends Autonomous{

  public enum states {
    START, START_FORWARD,             // Go forward off the platform 48 inches
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

  states nextState, currentState;
  WPI_TalonSRX leftMasterTalon, rightMasterTalon;
  int cyclesInState;
  double baseSpeed = 0.5; 
  double inchesTraveledInState;
  String waitingFor;

  public DoubleHatchAuto(WPI_TalonSRX left, WPI_TalonSRX right) {
    currentState = states.START;
    rightMasterTalon = right;
    leftMasterTalon = left;
  }
  
  @Override
  public void reset() {
    currentState = states.START;
    
  }
  
  @Override
  public void update() {
    switch (currentState) {
      case START:
      currentState = states.START_FORWARD;
      break;

      case START_FORWARD:
      resetDriveEncoders();
      //Go forward 48 inches
       //if(){ };
       
        drivePower(0.5);
        waitingFor = "Go forward 48 Inches";
      break;

      case ROTATE1_TO_ROCKET1:
        driveGyrolock(0, -30);
        if(gyroTurnDone()){
          currentState = states.ALIGN1;
        }
        waitingFor = "Put gyro heading to -30";
    
      break;

      case ALIGN1:
        drivePower(getCorrection(), -(getCorrection()));
        if(Math.abs(getCorrection()) <= 0.07 && isTargetInView()){
          currentState = states.FORWARD_TO_TARGET1;
        }
        waitingFor = "Target to Center";
      break;

    case FORWARD_TO_TARGET1:
      drivePower(0.5 - getCorrection(), 0.5 + getCorrection());
      if(collision()){
        currentState = states.PLACE_HATCH1;
      }
      waitingFor = "Collision";
      break;

    case PLACE_HATCH1:
      startTimer();
      drivePower(0);
      if(getTime() >= 1){
        resetTimer();
        currentState = states.BACKUP_AFTER_PLACING_HATCH;
      }
      waitingFor = "Waiting 1 second";
      break;

    case BACKUP_AFTER_PLACING_HATCH:
      resetDriveEncoders();
      drivePower(-0.5);
      //if (inchesTraveledInState <= -30)
        nextState = states.TURN_AROUND;
      waitingFor = "Back up 30 inches";
      break;

    case TURN_AROUND:
      driveGyrolock(0, -180);
      if(gyroTurnDone()){
        nextState = states.DRIVE_TO_LOADING_STATION;
      }
      waitingFor = "Put gyro heading to -180"; 
      break;

    case DRIVE_TO_LOADING_STATION:
      drivePower(0.5 - getCorrection(), 0.5 + getCorrection());
      if(collision()){
        nextState = states.RETREIVE_HATCH2;
      }
      waitingFor = "Collision";
      break;

    case RETREIVE_HATCH2:
      drivePower(0);
      startTimer();
      if(getTime() >= 1) {
        resetTimer();
        currentState = states.BACK_FROM_LOADING_STATION;
      }
      waitingFor = "Waiting 1 second";
      break;

    case BACK_FROM_LOADING_STATION:
    resetDriveEncoders();
     drivePower(-0.5);
     //if (inchesTraveledInState <= -276)
     nextState = states.ROTATE2_TO_ROCKET;
      waitingFor = "Go back 276 inches";
      break;

    case ROTATE2_TO_ROCKET:
      driveGyrolock(0, -150);
      if(gyroTurnDone()){
        currentState = states.ALIGN2;
      }
      waitingFor = "Put gyro heading to -150";
      break;

    case ALIGN2:
      drivePower(getCorrection(), -getCorrection());
      if(Math.abs(getCorrection()) <= 0.07 && targetInView())
        nextState = states.FORWARD_TO_TARGET2;
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
  }
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


  //public void update() {
    //inchesTraveledInState = (leftMasterTalon.getSelectedSensorPosition() + rightMasterTalon.getSelectedSensorPosition())/2/ticksPerInch;
    doTransition();
    currentState = nextState;
    doCurrentState(cameraX, cameraY);

    SmartDashboard.putString("state", currentState.toString());
    SmartDashboard.putString("Waiting for", waitingFor.toString());
  }
  public void resetState(){
    currentState = states.START;
    nextState = states.START;
      cyclesInState = 0;
      leftMasterTalon.setSelectedSensorPosition(0);
      rightMasterTalon.setSelectedSensorPosition(0);
  }
  public void doTransition() {
    if (currentState != nextState) {
      cyclesInState = 0;
      leftMasterTalon.setSelectedSensorPosition(0);
      rightMasterTalon.setSelectedSensorPosition(0);
    }
  }

  public void doCurrentState(double cameraX, double cameraY) {
    double leftSpeed = 0.3;
    double rightSpeed = 0.3;
    
    switch (currentState) {
    case START:
      break;

    case START_FORWARD:
      leftSpeed = baseSpeed;
      rightSpeed = baseSpeed;
      waitingFor = "Go forward 48 Inches";
      break;

    case ROTATE1_TO_ROCKET1:
      leftSpeed = baseSpeed;
      rightSpeed = -baseSpeed;
      waitingFor = "Put gyro heading to -30";
      break;

    case ALIGN1:
      leftSpeed = cameraX;
      rightSpeed = -cameraX;
      waitingFor = "Target to Center";
      break;

    case FORWARD_TO_TARGET1:
      leftSpeed = baseSpeed - cameraX;
      rightSpeed = baseSpeed + cameraX;
      waitingFor = "Collision";
      break;

    case PLACE_HATCH1:
      leftSpeed = 0;
      rightSpeed = 0;
      waitingFor = "Waiting 1 second";
      break;

    case BACKUP_AFTER_PLACING_HATCH:
      leftSpeed = -baseSpeed;
      rightSpeed = -baseSpeed;
      waitingFor = "Back up 30 inches";
      break;

    case TURN_AROUND:
      leftSpeed = baseSpeed;
      rightSpeed = -baseSpeed;
      waitingFor = "Put gyro heading to -180"; 
      break;

    case DRIVE_TO_LOADING_STATION:
      leftSpeed = baseSpeed - cameraX;
      rightSpeed = baseSpeed + cameraX;
      waitingFor = "Collision";
      break;

    case RETREIVE_HATCH2:
      leftSpeed = 0;
      rightSpeed = 0;
      waitingFor = "Waiting 1 second";
      break;

    case BACK_FROM_LOADING_STATION:
      leftSpeed = -baseSpeed;
      rightSpeed = -baseSpeed;
      waitingFor = "Go back 276 inches";
      break;

    case ROTATE2_TO_ROCKET:
      leftSpeed = -baseSpeed;
      rightSpeed = baseSpeed;
      waitingFor = "Put gyro heading to -150";
      break;

    case ALIGN2:
      leftSpeed = cameraX;
      rightSpeed = -cameraX;
      waitingFor = "Align to center of target";
      break;

    case FORWARD_TO_TARGET2:
      leftSpeed = baseSpeed - cameraX;
      rightSpeed = baseSpeed + cameraX;
      waitingFor = "Collision";
      break;

    case STOP:
      leftSpeed = 0;
      rightSpeed = 0;
      waitingFor = "We stopped";
      break;
    }
    leftMasterTalon.set(ControlMode.PercentOutput, leftSpeed);
    rightMasterTalon.set(ControlMode.PercentOutput, rightSpeed);

  }

  public void calcNextState(boolean targetValid, double cameraX, double cameraY,
      double gyroAngle, boolean collision, boolean align) {
    nextState = currentState;
    switch (nextState) {
    case START:
      
      break;
    case START_FORWARD:
      if (inchesTraveledInState >= 48)
        nextState = states.ROTATE1_TO_ROCKET1;
      break;

    case ROTATE1_TO_ROCKET1:
      if (gyroAngle <= -30.0)
        nextState = states.ALIGN1;
      break;

    case ALIGN1:
      if (Math.abs(cameraX) <= 0.07 && targetValid)
        nextState = states.FORWARD_TO_TARGET1;
      break;

    case FORWARD_TO_TARGET1:
      if (collision)
        nextState = states.PLACE_HATCH1;
      break;

    case PLACE_HATCH1:
      if (cyclesInState >= 50)
        nextState = states.BACKUP_AFTER_PLACING_HATCH;
      break;

    case BACKUP_AFTER_PLACING_HATCH:
      if (inchesTraveledInState <= -30)
        nextState = states.TURN_AROUND;
      break;

    case TURN_AROUND:
      if (gyroAngle <= -180)
        nextState = states.DRIVE_TO_LOADING_STATION;
      break;

    case DRIVE_TO_LOADING_STATION:
      if (collision)
        nextState = states.RETREIVE_HATCH2;
      break;

    case RETREIVE_HATCH2:
      if (cyclesInState >= 50)
        nextState = states.BACK_FROM_LOADING_STATION;
      break;

    case BACK_FROM_LOADING_STATION:
      if (inchesTraveledInState <= -276)
        nextState = states.ROTATE2_TO_ROCKET;
      break;

    case ROTATE2_TO_ROCKET:
      if (gyroAngle >= -150)
        nextState = states.ALIGN2;
      break;

    case ALIGN2:
      if (Math.abs(cameraX) <= 0.07 && targetValid)
        nextState = states.FORWARD_TO_TARGET2;
      break;

    case FORWARD_TO_TARGET2:
      if (collision)
        nextState = states.STOP;
      break;

    case STOP:
      break;
    }

  }
}

