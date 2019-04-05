package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
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
        PICKUP_FROM_STATION,
        REVERSE_OUTTAKE,
        INTAKE_DOWN,
        DRIVE_BACK,
        DONE,
        OVERRIDE
    }

    private Transmission intake, outtake;
    private DoubleSolenoid pivot;

    private double intakeSpeed, outtakeSpeed;

    private Value pivotState = Constants.kIntakeUp;

    private boolean intakeRequest, placeRequest, vomitRequest, pickupFromStationRequest;
    private boolean lastIntakeRequest, lastPlaceRequest, lastVomitRequest, lastPickupFromStationRequest;

    private boolean loadingBall = false, hasBall = false, runningIntake;

    private boolean lastCargoInCarriage;

    private AnalogInput intakeCargoSensor = new AnalogInput(0), elevatorCargoSensor = new AnalogInput(1);
    private DigitalInput raisedSensor = new DigitalInput(4);
    private IntakeState currentIntakeState = IntakeState.WAITING;
    private Elevator elevator = Robot.elevator;
    private Timer timer = new Timer();
    SpeedControllerMode intakeControlMode = SpeedControllerMode.kDutyCycle;
    SpeedControllerMode outtakeControlMode = SpeedControllerMode.kDutyCycle;

    public CargoIntake(Transmission intake, Transmission outtake, DoubleSolenoid pivotPiston) {
        this.intake = intake;
        this.outtake = outtake;
        pivot = pivotPiston;
    }

    @Override
    public void update() {
        if((!placeRequest && lastPlaceRequest) || 
         (!vomitRequest && lastVomitRequest) ||
         (!pickupFromStationRequest && lastPickupFromStationRequest)) {
                currentIntakeState = IntakeState.WAITING;
        }
        setIntakeSpeed(0);
        setOuttakeSpeed(0);
        
        switch (currentIntakeState) {
            case WAITING:
                loadingBall = false;
                pivotState = Constants.kIntakeUp;
                if(vomitRequest && !lastVomitRequest) {
                    currentIntakeState = IntakeState.VOMIT;
                }
                if(placeRequest && !lastPlaceRequest) {
                    currentIntakeState = IntakeState.PLACE;
                }
                if(intakeRequest && !lastIntakeRequest) {
                    if(getCargoIntakeSensor()) {
                        currentIntakeState = IntakeState.TRANSFERRING;
                    }
                    if(getCargoCarriageSensor()) {
                        currentIntakeState = IntakeState.WAITING;
                        break;
                    }
                    currentIntakeState = IntakeState.INTAKING;
                    loadingBall = true;
                }
                if(getCargoCarriageSensor()) {
                    hasBall = true;
                    timer.reset();
                    timer.stop();
                    if(hasBall) {
                        setOuttakeSpeed(-.2);
                    }
                }
                else if(hasBall && !getCargoCarriageSensor()) {
                    setOuttakeSpeed(.2);
                    if(lastCargoInCarriage) 
                        timer.start();
                    if(timer.get() >= 5)
                        hasBall = false;
                }
                if(pickupFromStationRequest && !lastPickupFromStationRequest) {
                    elevator.set(Constants.kElevatorCargoStationPickup);
                    currentIntakeState = IntakeState.PICKUP_FROM_STATION;
                }
                break;
            case INTAKING:
                if(!intakeRequest) {
                    currentIntakeState = IntakeState.WAITING;
                    setIntakeDown(false);
                    break;
                }
                setIntakeDown(true);
                if(runningIntake)
                    setIntakeSpeed(.9);
                elevator.set(Constants.kElevatorBallIntake);
                if(getCargoIntakeSensor() && intakeRequest) {
                    setIntakeDown(false);
                    currentIntakeState = IntakeState.RAISING;
                }
                break;
            case RAISING:
                pivot.set(Value.kOff);
                setIntakeSpeed(0);
                if(elevator.inPosition() && getIsUp()) {
                    currentIntakeState = IntakeState.TRANSFERRING;
                }
                break;
            case TRANSFERRING:
                setIntakeDown(false);
                setOuttakeSpeed(.35);
                setIntakeSpeed(.6);
                if(getCargoCarriageSensor()) {
                    hasBall = true;
                    currentIntakeState = IntakeState.DONE;
                }
                break;
            case PICKUP_FROM_STATION:
                setOuttakeSpeed(-.25);
                if(getCargoCarriageSensor()) {
                    elevator.set(Constants.kElevatorHatchPickup);
                    currentIntakeState = IntakeState.DONE;
                }
                break;
            case PLACE:
                hasBall = false;
                setOuttakeSpeed(1);
                break;
            case REVERSE_OUTTAKE:
                setOuttakeSpeed(-.25);
                break;
            case VOMIT:
                setIntakeSpeed(-1);
                break;
            case INTAKE_DOWN:
                setIntakeDown(true);
                break;
            case DRIVE_BACK:
                setIntakeDown(true);
                setIntakeSpeed(1);
                break;
            case DONE:
                setIntakeDown(false);
                currentIntakeState = IntakeState.WAITING;
                break;
            case OVERRIDE:
                setIntakeDown(true);
                setIntakeSpeed(1);
                break;
        }
        
        lastIntakeRequest = intakeRequest;
        lastPlaceRequest = placeRequest;
        lastVomitRequest = vomitRequest;
        lastPickupFromStationRequest = pickupFromStationRequest;
        lastCargoInCarriage = getCargoCarriageSensor();

        pivot.set(pivotState);
        intake.set(intakeSpeed, intakeControlMode);
        outtake.set(outtakeSpeed, outtakeControlMode);
    }

    /**
     * @param desiredSpeed the desiredSpeed to set
     */
    private void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }
    private void setOuttakeSpeed(double speed) {
        this.outtakeSpeed = speed;
    }

    private void setIntakeDown(boolean down) {
        if(down) {
            pivotState = Constants.kIntakeDown;
        }
        else {
            pivotState = Constants.kIntakeUp;
        }
    }
    /**
     * @param currentIntakeState the currentIntakeState to set
     */
    public void setIntakeState(IntakeState mode) {
        currentIntakeState = mode;
    }

    /**
     * @return the currentIntakeState
     */
    public IntakeState getIntakeState() {
        return currentIntakeState;
    }
    /**
     * @param intakeRequest the intakeRequest to set
     */
    public void setIntakeRequest(boolean intakeRequest) {
        this.intakeRequest = intakeRequest;
    }
    /**
     * @param placeRequest the placeRequest to set
     */
    public void setPlaceRequest(boolean placeRequest) {
        this.placeRequest = placeRequest;
    }

    /**
     * @param vomitRequest the vomitRequest to set
     */
    public void setVomitRequest(boolean vomitRequest) {
        this.vomitRequest = vomitRequest;
    }

    /**
     * @param pickupFromStationRequest the pickupFromStationRequest to set
     */
    public void setPickupFromStationRequest(boolean pickupFromStationRequest) {
        this.pickupFromStationRequest = pickupFromStationRequest;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Cargo Intake IntakeState", getIntakeState().toString());
        
    }

    @Override
    public void resetSensors() {
        setIntakeState(IntakeState.WAITING);
    }

    public boolean getCargoIntakeSensor() {
        return intakeCargoSensor.getVoltage() < Constants.kOpticalSensorVoltageThreshold && intakeCargoSensor.getVoltage() != 0;
    }

    public boolean getCargoCarriageSensor() {
        return elevatorCargoSensor.getVoltage() < Constants.kOpticalSensorVoltageThreshold && elevatorCargoSensor.getVoltage() != 0;
    }
    public boolean getIsUp() {
        return !raisedSensor.get();
    }
    public boolean getLoadingBall() {
        return loadingBall;
    }
    public void stopLoadingBall() {
        loadingBall = false;
    }
    public void setRunIntake(boolean run) {
        runningIntake = run;
    }

    public boolean isDone() {
        return currentIntakeState == IntakeState.DONE;
    }

    public boolean hasBall() {
        return hasBall;
    }

    @Override
    public void debug() {
    SmartDashboard.putNumber("Cargo intake sensor voltage", intakeCargoSensor.getVoltage());
    SmartDashboard.putNumber("Elevator Cargo sensor Voltage", elevatorCargoSensor.getVoltage());
    SmartDashboard.putBoolean("Ball in Intake", getCargoIntakeSensor());
    SmartDashboard.putBoolean("Ball in Carriage", getCargoCarriageSensor());
    SmartDashboard.putBoolean("Cargo Intake Raised", getIsUp());
    SmartDashboard.putNumber("Cargo Intake current", intake.getOutputCurrent(0));
    SmartDashboard.putNumber("Cargo Intake speed", intakeSpeed);

    }
}