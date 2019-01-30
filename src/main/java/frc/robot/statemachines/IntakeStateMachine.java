package frc.robot.statemachines;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class IntakeStateMachine {
    public enum States {
        WAITING,
        INTAKING,
        DROPPING,
        RAISING,
        TRANSFER
    } 

    CargoIntake intake;
    Elevator elevator;

    private States currentState = States.WAITING;

    private boolean intakeRequest;


    public IntakeStateMachine() {
        intake = Robot.cargoIntake;
        elevator = Robot.elevator;
    }

    public void update() {

        switch(currentState) { 
            case WAITING:
                if(intakeRequest) {
                    currentState = States.INTAKING;
                }
                break;
            case INTAKING:
                intake.setIntakeState(IntakeState.INTAKING);
                elevator.set(Constants.kElevatorLevel1);
                if(intake.getHasCargo()) {
                    currentState = States.RAISING;
                }
                break;
            case RAISING:
                intake.setIntakeState(IntakeState.RAISING);
                if(elevator.inPosition()
                break;
            case TRANSFER:
                break;
        }

    }

    public void outputToSmartDashboard() {

    }
}