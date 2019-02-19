package frc.robot.statemachines;

public abstract class StateMachine {

    protected boolean request, lastRequest;

    public void setRequest(boolean req) {
        request = req;
    }

    public abstract void update();

    public abstract void outputToSmartDashboard();



}