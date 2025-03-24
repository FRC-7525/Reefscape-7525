package frc.robot.Subsystems.Coraler;

import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import frc.robot.SubsystemManager.SubsystemManager;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum CoralerStates implements SubsystemStates {
	CORALING("Coraling", () -> {
		return (SubsystemManager.getInstance().getDriverReefScoringLevel() == 4 || SubsystemManager.getInstance().getOperatorReefScoringLevel() == 4) ? CORALING_VELOCITY_L4 : SubsystemManager.getInstance().getOperatorReefScoringLevel() != 1 && SubsystemManager.getInstance().getDriverReefScoringLevel() != 1 ? CORALING_VELOCITY_L3_L2 : CORALING_VELOCITY_L1;
	}),
	INAKING("Inaking", () -> INTAKING_VELOCITY),
	CENTERING("Centering", () -> CENTERING_VELOCITY),
	IDLE("Stopped", () -> IDLE_VELOCITY);

	private String stateString;
	private Supplier<Double> velocitySupplier;

	CoralerStates(String stateString, Supplier<Double> velocitySupplier) {
		this.stateString = stateString;
		this.velocitySupplier = velocitySupplier;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public Supplier<Double> getVelocitySupplier() {
		return velocitySupplier;
	}
}
