package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.motion.Path;
import frc.robot.motion.PathFollower;
import frc.robot.motion.PathList;
import frc.robot.statemachines.CargoIntakeStateMachine;
import frc.robot.statemachines.HatchIntakeStateMachine;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Drive.DriveMode;

public abstract class Action {

	private Drive drive = Robot.drive;
	private CargoIntake cargo = Robot.cargoIntake;
	private HatchMechanism hatch = Robot.hatch;
	private HumanInput HI = Robot.HI;
	private Camera camera = Robot.camera;
	private HatchIntakeStateMachine hatchIntakeStateMachine = Robot.hatchIntakeStateMachine;
	private CargoIntakeStateMachine cargoIntakeStateMachine = Robot.cargoIntakeStateMachine;
	
	private PathFollower pathFollower = new PathFollower();
	private Timer timer = new Timer();

	protected boolean isDone;

	protected Enum<?> currentState;

	public abstract void update();
	
	public abstract Enum<?> getState();

	public abstract void init();
    
    public boolean isDone() {
		return isDone;
	}
	
	protected void resetSensors() {
		drive.resetSensors();
	}
	
	protected void resetDriveEncoders() {
		drive.resetDriveEncoders();
    }
	
	//gear
	
	protected void setHighGear(boolean highGear) {
		drive.setHighGear(highGear);
	}
	
	protected void drivePower(double speed) {
		drive.setDriveMode(DriveMode.OPEN_LOOP);
		drive.set(speed, speed);
	}
	
	protected void drivePower(double leftSpeed, double rightSpeed) {
		drive.setDriveMode(DriveMode.OPEN_LOOP);
		drive.set(leftSpeed, rightSpeed);
	}
	
	protected double getDesiredAngle() {
		return drive.getDesiredAngle();
	}
	
	protected double getAngle() {
		return drive.getAngle();
	}
	
	protected void driveGyrolock(double desiredSpeed, double desiredAngle) {
		setHighGear(true);
		drive.setDriveMode(DriveMode.GYROLOCK);
		drive.set(desiredSpeed, desiredSpeed);
		drive.setDesiredAngle(desiredAngle);
	}
	
	protected void drivePosition(double position) {
		drive.setDriveMode(DriveMode.POSITION);
		drive.set(position, position);
	}

	protected boolean gyroTurnDone() {
		return drive.gyroInPosition();
	}
	protected boolean driveInPosition() {
		return drive.driveInPosition();
	}
	protected boolean targetInView() {
		return camera.isTargetInView();
	}
	protected boolean isAligned() {
		return (camera.getCorrection() < .07);
	}
	//motion profiling
	protected void startPathFollower(Path path) {
		pathFollower.followPath(path);
	}
	protected boolean isPathDone() {
		return pathFollower.isDone();
	}
	protected Path getPath(String path) {
		return PathList.getPath(path);
	}
	
	public void stopPathFollower() {
		pathFollower.stop();
	}
	//timer
	
	public void resetTimer() {
		timer.stop();
		timer.reset();
	}
	public void startTimer() {
		timer.start();
	}
	public double getTime() {
		return timer.get();
	}

	protected boolean collision(){
		return drive.collision();
	}
	protected double getCorrection(){
		return camera.getCorrection();
	}
	protected double getTargetHorizError(){
		return camera.getTargetHorizError();
	}
	protected boolean isTargetInView(){
		return camera.isTargetInView();
	}
	public double getAveragePosition(){
		return drive.getAveragePosition();
	}
	protected double getAccelerometer(){
		return drive.getAcceleration();
	}
	protected String getStartPos() {
		return HI.getLeftRightCenter();
	}
	protected boolean hasHatch() {
		return hatch.getHasHatch();
	}
	protected boolean hasCargo() {
		return cargo.getHasCargo(); 
	}
	protected void setHatchIntakeRequest(boolean intakeRequest) {
		hatchIntakeStateMachine.setIntakeRequest(intakeRequest);
	}
	protected void setCargoIntakeRequest(boolean intakeRequest) {
		cargoIntakeStateMachine.setIntakeRequest(intakeRequest);
	}

}