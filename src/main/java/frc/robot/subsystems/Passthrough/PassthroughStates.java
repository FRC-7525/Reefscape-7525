package frc.robot.Subsystems.Passthrough;

import static frc.robot.Subsystems.Coraler.CoralerConstants.OUTTAKING_VELOCITY;
import static frc.robot.Subsystems.Passthrough.PassthroughConstants.INTAKING_VELOCITY;
import static frc.robot.Subsystems.Passthrough.PassthroughConstants.OFF_VELOCITY;

import org.team7525.subsystem.SubsystemStates;

public enum PassthroughStates implements SubsystemStates {
	OFF("Off", OFF_VELOCITY),
	INTAKING("Intaking", INTAKING_VELOCITY),
	OUTTAKING("OUTTAKING", OUTTAKING_VELOCITY);

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
