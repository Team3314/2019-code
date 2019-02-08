package frc.robot.actions;

public class CargoPlaceRocket extends Action{

    public enum State {
        TURN,
    }

    private State currentState;

    public CargoPlaceRocket(int level) {
        
    }

    @Override
    public void update() {

    }

    @Override
    public Enum<State> getState() {
        return currentState;
    }

    @Override
    public void init() {

    }

}