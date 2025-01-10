package frc.robot.Subsystems.AutoAlign;

import frc.robot.Subsystems.Manager.ManagerStates;
import org.team7525.subsystem.SubsystemStates;

// TODO CHANGE MANAGER STATE ONCE MORE ADDED
public enum AutoAlignStates implements SubsystemStates {
	IDLE("Idle", ManagerStates.IDLE),
	DRIVING_REEF_L1("Driving to Reef L1", ManagerStates.IDLE),
	DRIVING_REEF_L2("Driving to Reef L2", ManagerStates.IDLE),
	DRIVING_REEF_L3("Driving to Reef L3", ManagerStates.IDLE),
	DRIVING_REEF_L4("Driving to Reef L4", ManagerStates.IDLE),
	DRIVING_CORAL("Driving to Coral Station", ManagerStates.IDLE);

	AutoAlignStates(String stateString, ManagerStates managerState) {
		this.stateString = stateString;
		this.managerState = managerState;
	}

	private String stateString;
	private ManagerStates managerState;

	@Override
	public String getStateString() {
		return stateString;
	}

	protected ManagerStates getManagerState() {
		return managerState;
	}
}
