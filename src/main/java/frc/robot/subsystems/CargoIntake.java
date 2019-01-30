package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.infrastructure.SpeedControllerMode;
import frc.robot.infrastructure.Transmission;

public class CargoIntake implements Subsystem {

    public enum IntakeState {
        WAITING,
        INTAKING,
        RAISING,
        TRANSFERRING,
        VOMIT,
        PLACE,
        OVERRIDE
    }

    private Transmission intake, outtake;
    /* TODO Pick motor or piston
    private EncoderTransmission pivot;
    private DoubleSolenoid pivot;
    */
    private double intakeSpeed, outtakeSpeed;

    private boolean hasCargo, elevatorInPosition, intakeCommand, outtakeCommand, cargoInCarriage;

    private DigitalOutput cargoSensor = new DigitalOutput(0);
    private IntakeState currentIntakeState = IntakeState.WAITING;
    SpeedControllerMode intakeControlMode = SpeedControllerMode.kDutyCycle;
    SpeedControllerMode outtakeControlMode = SpeedControllerMode.kDutyCycle;

    public CargoIntake(Transmission transmission) {
        intake = transmission;
    }

    @Override
    public void update() {
        switch (currentIntakeState) {
            case WAITING:
                setIntakeUp();
                setIntakeSpeed(0);
                break;
            case INTAKING:
                setIntakeDown();
                setIntakeSpeed(1);
                break;
            case RAISING:
                setIntakeUp();
                setIntakeSpeed(0);
                break;
            case TRANSFERRING:
                setIntakeSpeed(1);
                setOuttakeSpeed(1);
                break;
            case PLACE:
                setOuttakeSpeed(1);
                break;
            case VOMIT:
                setIntakeSpeed(-1);
                break;
            case OVERRIDE:
                setIntakeDown();
                setIntakeSpeed(1);
                break;
        }
        intake.set(intakeSpeed, intakeControlMode);
        outtake.set(outtakeSpeed, outtakeControlMode);

    }

    /**
     * @param desiredSpeed the desiredSpeed to set
     */
    public void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }
    public void setOuttakeSpeed(double speed) {
        this.outtakeSpeed = speed;
    }

    public void setIntakeDown() {

    }

    public void setIntakeUp() {

    }

    /**
     * @param currentState the currentState to set
     */
    public void setIntakeState(IntakeState mode) {
        currentIntakeState = mode;
    }

    /**
     * @return the currentState
     */
    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Intake current", intake.getOutputCurrent(0));
        SmartDashboard.putNumber("Intake speed", intakeSpeed);
        SmartDashboard.putString("Intake state", getIntakeState().toString());
    }

    @Override
    public void resetSensors() {
        setIntakeState(IntakeState.WAITING);
    }


    public void setElevatorInPosition(boolean elevator){
        elevatorInPosition = elevator;
    }

    public void setIntake(boolean intake) {

    }

    public boolean getHasCargo() {
        return hasCargo;
    }

    public boolean getCargoInCarriage() {
        return cargoInCarriage;
    }
}