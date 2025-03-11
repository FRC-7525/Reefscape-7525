package frc.robot.Subsystems.Passthrough;

import org.team7525.subsystem.SubsystemStates;

public enum PassthroughStates implements SubsystemStates {
	OFF("Off", 0),
	INTAKING("Intaking", 0);

	private String stateString;
	private double velocity;

	PassthroughStates(String stateString, double velocity) {
		this.stateString = stateString;
		this.velocity = velocity;
	}

	public double getVelocity() {
		return velocity;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
