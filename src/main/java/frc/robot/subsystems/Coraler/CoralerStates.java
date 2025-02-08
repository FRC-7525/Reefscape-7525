package frc.robot.Subsystems.Coraler;

import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import org.team7525.subsystem.SubsystemStates;

public enum CoralerStates implements SubsystemStates {
	CORALING("Coraling", CORALING_VELOCITY),
	INAKING("Inaking", INTAKING_VELOCITY),
	CENTERING("Centering", CENTERING_VELOCITY),
	IDLE("Stopped", IDLE_VELOCITY);

	private String stateString;
	private double velocity;

	CoralerStates(String stateString, double velocity) {
		this.stateString = stateString;
		this.velocity = velocity;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getVelocity() {
		return velocity;
	}
}
