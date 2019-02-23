package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CargoIntake.IntakeState;

public class ClimberStateMachine extends StateMachine {
    public enum State {
        WAITING,
        RAISE_BACK,
        RAISE_FRONT,
        DRIVE,
        DONE
    }

    private Superstructure superstructure = Robot.superstructure;
    private Climber climber = Robot.climber;
    private CargoIntake cargoIntake  = Robot.cargoIntake;
    private Elevator elevator = Robot.elevator;
    private HumanInput HI = Robot.HI;

    private State currentState = State.WAITING;

    @Override
    public void update() {
        switch(currentState) { 
            case WAITING:
                climber.setClimberDown(false);
                if(HI.getClimb()) {
                    currentState = State.RAISE_BACK;
                    cargoIntake.setIntakeState(IntakeState.INTAKE_DOWN);
                    elevator.set(Constants.kElevatorBallLevel1);
                }
                break;
            case RAISE_BACK:
                break;
            case RAISE_FRONT:
                break;
            case DRIVE:
                break;
            case DONE:
                break;
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Climber State Machine State", currentState.toString());
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }
}