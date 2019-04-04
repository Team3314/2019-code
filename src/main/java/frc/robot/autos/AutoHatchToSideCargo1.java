/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.statemachines.GamePieceStateMachine;
import frc.robot.statemachines.GamePieceStateMachine.GamePieceStateMachineMode;
import frc.robot.subsystems.Drive.DriveMode;

/**
 * Add your docs here.
 */
public class AutoHatchToSideCargo1 extends Autonomous{

    public enum states{
        START,
        START_FORWARD, //Go forward 17 feet
        ROTATE_TO_CARGO, //Rotate to gyro heading of 90
        PLACE_HATCH,
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
                driveGyrolock(0.5, getAngle());
                if(getAveragePosition() > 204){
                    currentState = states.ROTATE_TO_CARGO;
                }
              break;
            case ROTATE_TO_CARGO:
                if(getStartPos().equals("StartR")){
                    driveGyrolock(0, -90, DriveMode.GYROLOCK);
                }
                else if(getStartPos().equals("StartL")){
                    driveGyrolock(0, 90, DriveMode.GYROLOCK);
                }
                if(gyroTurnDone()){
                    currentState = states.PLACE_HATCH;
                }
              break;
            case PLACE_HATCH:
                gamePieceInteract(GamePieceStateMachineMode.CARGO_SHIP, .5);
                if(gamePieceStateMachine.isDone()){
                  stopGamePieceInteract();
                  currentState = states.STOP;
                }
              break;

            case STOP:
              break;
        }
        SmartDashboard.putString("Auto State", currentState.toString());
  }
}
