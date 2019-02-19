package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class CargoIntakeStateMachine extends StateMachine {
    public enum States {
        WAITING,
        INTAKING,
        RAISING,
        TRANSFER
    } 

    CargoIntake intake;
    Elevator elevator;
    HumanInput HI;

    private States currentState = States.WAITING;


    public CargoIntakeStateMachine() {
        intake = Robot.cargoIntake;
        elevator = Robot.elevator;
        HI = Robot.HI;
    }
    @Override
    public void update() {
        if(!request) {
            currentState = States.WAITING;
        }
        switch(currentState) { 
            case WAITING:
                intake.setIntakeState(IntakeState.WAITING);
                if(request && !lastRequest) {
                    currentState = States.INTAKING;
                }
                break;
            case INTAKING:
                intake.setIntakeState(IntakeState.INTAKING);
                elevator.set(Constants.kElevatorBallLevel1);
                if(intake.getCargoInIntake()) {
                    currentState = States.RAISING;
                }
                break;
            case RAISING:
                intake.setIntakeState(IntakeState.RAISING);
                if(elevator.inPosition() && intake.getIsUp()) {
                    currentState = States.TRANSFER;
                }
                break;
            case TRANSFER:
                intake.setIntakeState(IntakeState.TRANSFERRING);
                if(intake.getCargoInCarriage()) {
                    currentState = States.WAITING;
                }
                break;
        }
        lastRequest = request;
    }
    
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Cargo Intake State Machine State", currentState.toString());
    }
}