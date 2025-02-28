package frc.robot.Subsystems.LED;

import org.team7525.subsystem.SubsystemStates;

public enum LEDStates implements SubsystemStates {
	DISABLED("Disabled", 400),
	IDLE("Idle", 800),
	INTAKING("Intaking", 1200),
	SCORING("Scoring", 1600),
	AUTOALIGN("Autoalign", 2000),
	ALGAE("Algae", 2400),
	CLIMBING("Climbing"),
	L1("L1", 2800),
	L2("L2", 3200),
	L3("L3", 3600),
	L4("L4", 4000); //TODO: Probably need to change these values cuz rez is too low

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
