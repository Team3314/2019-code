package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.HumanInput;
import frc.robot.Robot;
import frc.robot.motion.Path;
import frc.robot.motion.PathFollower;
import frc.robot.motion.PathList;
import frc.robot.statemachines.GamePieceStateMachine;
import frc.robot.statemachines.TrackingStateMachine;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveMode;

public abstract class Autonomous {

    private Drive drive = Robot.drive;
	private HumanInput HI = Robot.HI;
	private Camera camera = Robot.camera;
	protected GamePieceStateMachine gamePieceStateMachine = Robot.gamePieceStateMachine;
	protected TrackingStateMachine trackingStateMachine= Robot.trackingStateMacahine;
	
	private PathFollower pathFollower = new PathFollower();
	private Timer timer = new Timer();
	
	public abstract void reset();

	public abstract void update();
	
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
		drive.setDriveMode(DriveMode.TANK);
		drive.set(speed, speed);
	}
	
	protected void drivePower(double leftSpeed, double rightSpeed) {
		drive.setDriveMode(DriveMode.TANK);
		drive.set(leftSpeed, rightSpeed);
	}
	
	protected void turnMotionMagic(double desiredAngle) {
		setHighGear(false);
		drive.setDesiredAngle(desiredAngle);
	}
	
	protected double getDesiredAngle() {
		return drive.getDesiredAngle();
	}
	
	protected boolean isTurnDone() {
		return getAngle() <= getDesiredAngle() + 5 && getAngle() >= getDesiredAngle() - 5;
	}
	
	protected double getAngle() {
		return drive.getAngle();
	}
	
	protected void driveGyrolock(double desiredSpeed, double desiredAngle) {
		drive.setDriveMode(DriveMode.GYROLOCK);
		setHighGear(true);
		drive.set(desiredSpeed, desiredSpeed);
		drive.setDesiredAngle(desiredAngle);
	}

	protected void driveVision(double desiredSpeed) {
		drive.setDriveMode(DriveMode.VISION_CONTROL);
		drive.set(desiredSpeed, desiredSpeed);
	}
	
	protected boolean gyroTurnDone() {
		return drive.gyroInPosition();
	}
	protected boolean targetInView() {
		return camera.isTargetInView();
	}
	protected double getCameraDistance() {
		return camera.getRawDistance();
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
	//Double Hatch Auto

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
		return drive.getAverageNeoPosition();
	}
	protected double getAccelerometer(){
		return drive.getAcceleration();
	}
	protected String getStartPos() {
		return HI.getLeftRightCenter();
	}

	protected void setGamePieceRequest(boolean request) {
		gamePieceStateMachine.setRequest(request);
		if(request) {
			drive.set(.5, .5);
		}
	}
}