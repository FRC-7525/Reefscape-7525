package frc.robot.Subsystems.Coraler;

import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import frc.robot.SubsystemManager.SubsystemManager;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum CoralerStates implements SubsystemStates {
	CORALING("Coraling", () -> {
		return (SubsystemManager.getInstance().getOperatorReefScoringLevel() == 4 || SubsystemManager.getInstance().getDriverReefScoringLevel() == 4) ? CORALING_VELOCITY_L4 : CORALING_VELOCITY_REGULAR;
	}),
	INAKING("Inaking", () -> INTAKING_VELOCITY),
	CENTERING("Centering", () -> CENTERING_VELOCITY),
	IDLE("Stopped", () -> IDLE_VELOCITY);

	private String stateString;
	private double velocity;

	CoralerStates(String stateString, Supplier<Double> velocitySupplier) {
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
