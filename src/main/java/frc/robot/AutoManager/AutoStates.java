package frc.robot.AutoManager;

import frc.robot.SubsystemManager.SubsystemManagerStates;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum AutoStates implements SubsystemStates {
	INTAKING_CORAL("Intaking Coral", () -> SubsystemManagerStates.INTAKING_CORALER),
	SCORING_CORAL("Scoring Coral", () -> SubsystemManagerStates.AUTO_ALIGN_FAR),
	IDLE("Idle", () -> SubsystemManagerStates.IDLE);

	Supplier<SubsystemManagerStates> state;
	String stateString;

	AutoStates(String stateString, Supplier<SubsystemManagerStates> state) {
		this.state = state;
		this.stateString = stateString;
	}

	public String getStateString() {
		return stateString;
	}

	public Supplier<SubsystemManagerStates> getManagerState() {
		return state;
	}
}
