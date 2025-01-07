package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.team7525.subsystem.SubsystemStates;

public enum CoralerStates implements SubsystemStates {
	CORALING("Coraling", RotationsPerSecond.of(2)),
	INAKING("Inaking", RotationsPerSecond.of(0)),
	IDLE("Stopped", RotationsPerSecond.of(0));

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
