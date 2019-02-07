package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class CargoIntakeStateMachine {
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

    private boolean intakeRequest, lastIntakeRequest;


    public CargoIntakeStateMachine() {
        intake = Robot.cargoIntake;
        elevator = Robot.elevator;
        HI = Robot.HI;
    }

    public void update() {
        if(!intakeRequest) {
            currentState = States.WAITING;
        }
        switch(currentState) { 
            case WAITING:
                intake.setIntakeState(IntakeState.WAITING);
                if(intakeRequest && !lastIntakeRequest) {
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
                if(elevator.inPosition()) {
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
        lastIntakeRequest = intakeRequest;
    }  

    public void setIntakeRequest(boolean request) {
        intakeRequest = request;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Intake State Machine State", currentState.toString());
    }
}