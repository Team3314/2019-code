/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class AutoHatchToSideCargo1 extends Autonomous{

    public enum states{
        START,
        START_FORWARD, //Go forward 17 feet
        ROTATE_TO_CARGO, //Rotate to gyro heading of 90
        ALIGN, //Align to center of first cargo 
        FORWARD_TO_CARGO, //Go towards cargo ship until collision
        PLACE_HATCH, //Simulate placing hatch for 1 second
        STOP
    }
    states currentState;
    String waitingFor = null;

    public AutoHatchToSideCargo1(){
        currentState = states.START;
    }

    @Override
    public void reset(){
        currentState = states.START;
    }

    @Override
    public void update(){
        switch(currentState){
            case START:
            currentState = states.START_FORWARD;
              break;
              
            case START_FORWARD:
            drivePower(0.5);
            if(getAveragePosition() > 204){
                currentState = states.ROTATE_TO_CARGO;
                if(getStartPos().equals("StartR")){
                    driveGyrolock(0, -90);
                }
                else if(getStartPos().equals("StartL")){
                    driveGyrolock(0, 90);
                }
            }
              break;

            case ROTATE_TO_CARGO:
            if(gyroTurnDone()){
                currentState = states.ALIGN;
            }
            if(getStartPos().equals("StartR"))
                waitingFor = "Put gyro heading to 90";
            else if(getStartPos().equals("StartL"))
                waitingFor = "Put gyro heading to -90";
              break;

            case ALIGN:
            drivePower(getCorrection(), -getCorrection());
            if(Math.abs(getCorrection()) <= 0.07 && targetInView())
                currentState = states.FORWARD_TO_CARGO;
              break;

            case FORWARD_TO_CARGO:
            drivePower(0.5 + getCorrection(), 0.5 - getCorrection());
            if (collision()) {
                currentState = states.PLACE_HATCH;
            startTimer();
            }
            waitingFor = "Collision";
              break;

              case PLACE_HATCH:
              drivePower(0);
              if(getTime() >= 1){
                  resetTimer();
                  currentState = states.STOP;
              }
              waitingFor = "1 sec for Hatch placing";
              break;

            case STOP:
              break;
        }
        SmartDashboard.putString("Waiting For", waitingFor);
        SmartDashboard.putString("State", currentState.toString());
        SmartDashboard.putBoolean("collision?", collision());
        SmartDashboard.putString("Starting Postion", getStartPos().toString());
  }
}
