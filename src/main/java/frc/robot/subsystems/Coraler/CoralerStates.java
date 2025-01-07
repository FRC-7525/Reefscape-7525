package frc.robot.Subsystems.Coraler;

import edu.wpi.first.units.measure.AngularVelocity;
import org.team7525.subsystem.SubsystemStates;

import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

public enum CoralerStates implements SubsystemStates {
	CORALING("Coraling", CORALING_VELOCITY),
	INAKING("Inaking", INTAKING_VELOCITY),
	IDLE("Stopped", IDLE_VELOCITY);

	private String stateString;
	private AngularVelocity velocity;

	CoralerStates(String stateString, AngularVelocity velocity) {
		this.stateString = stateString;
		this.velocity = velocity;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public AngularVelocity getVelocity() {
		return velocity;
	}
}
