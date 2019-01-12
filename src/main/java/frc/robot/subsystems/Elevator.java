package frc.robot.subsystems;

public class Elevator implements Subsystem {

    private static Elevator mInstance = new Elevator();

    public enum elevatorState {

    }

    public static Elevator getInstance() {
        return mInstance;
    }

    private elevatorState currentState;

    public void update(){
        switch(currentState) {

        }
    }

    public void outputToSmartDashboard(){

    }   

    public void resetSensors(){

    } 

}