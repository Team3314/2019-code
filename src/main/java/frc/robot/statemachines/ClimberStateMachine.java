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
    public enum States {
        WAITING,
        RAISE_BACK,
        RAISE_FRONT,
        DRIVE
    }

    private Superstructure superstructure = Robot.superstructure;
    private Climber climber = Robot.climber;
    private CargoIntake cargoIntake  = Robot.cargoIntake;
    private Elevator elevator = Robot.elevator;
    private HumanInput HI = Robot.HI;

    private States currentState = States.WAITING;

    @Override
    public void update() {
        switch(currentState) { 
            case WAITING:
                climber.setClimberDown(false);
                if(HI.getClimb()) {
                    currentState = States.RAISE_BACK;
                    cargoIntake.setIntakeState(IntakeState.STOP_DOWN);
                    elevator.set(Constants.kElevatorBallLevel1);
                }
                break;
            case RAISE_BACK:
                break;
            case RAISE_FRONT:
                break;
            case DRIVE:
                break;
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Cargo Intake State Machine State", currentState.toString());
    }
}