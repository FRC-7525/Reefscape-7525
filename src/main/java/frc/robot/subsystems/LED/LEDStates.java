package frc.robot.Subsystems.LED;

import org.team7525.subsystem.SubsystemStates;

public enum LEDStates implements SubsystemStates {
    RED("Red", 500),
    ORANGE("Orange", 1000),
    GREEN("Green", 1500),
    PURPLE("Purple", 2000),
    BLUE("Blue", 2500),
    YELLOW("Yellow", 3000),
    WHITE("White", 3500),
    OFF("Off", 3800);

    private int pwmSignal;
    private String stateString;

    LEDStates(String stateString, int pwmSignal) {
        this.pwmSignal = pwmSignal;
        this.stateString = stateString;
    }

    @Override
    public String getStateString() {
        return stateString;
    }
    
    public int getSignal() {
        return pwmSignal;
    }
}
