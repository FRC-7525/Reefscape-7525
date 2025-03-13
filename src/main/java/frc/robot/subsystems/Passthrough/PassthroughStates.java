package frc.robot.subsystems.Passthrough;

import static frc.robot.subsystems.Coraler.CoralerConstants.INTAKING_VELOCITY;
import static frc.robot.subsystems.Passthrough.PassthroughConstants.OFF_VELOCITY;

import org.team7525.subsystem.SubsystemStates;

public enum PassthroughStates implements SubsystemStates {
	OFF("Off", OFF_VELOCITY),
	INTAKING("Intaking", INTAKING_VELOCITY);

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
