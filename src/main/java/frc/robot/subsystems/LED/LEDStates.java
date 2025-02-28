package frc.robot.Subsystems.LED;

import org.team7525.subsystem.SubsystemStates;

public enum LEDStates implements SubsystemStates {
	DISABLED("Disabled", 186),
	IDLE("Idle", 559),
	INTAKING("Intaking", 932),
	SCORING("Scoring", 1304),
	AUTOALIGN("Autoalign", 1676),
	ALGAE("Algae", 2049),
	CLIMBING("Climbing", 2421),
	L1("L1", 2794),
	L2("L2", 3166),
	L3("L3", 3538),
	L4("L4", 3911); //TODO: Probably need to change these values cuz rez is too low

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
