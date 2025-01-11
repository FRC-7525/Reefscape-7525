package frc.robot.Subsystems.AutoAlign;

import frc.robot.Subsystems.Manager.ManagerStates;
import org.team7525.subsystem.SubsystemStates;

// TODO CHANGE MANAGER STATE ONCE MORE ADDED
public enum AutoAlignStates implements SubsystemStates {
	OFF("Idle"),
	DRIVING_REEF("Driving to Reef"),
	DRIVING_CORAL("Driving to Coral Station");

	AutoAlignStates(String stateString) {
		this.stateString = stateString;
	}

	private String stateString;

	@Override
	public String getStateString() {
		return stateString;
	}
}
