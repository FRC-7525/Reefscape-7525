package frc.robot.subsystems.Elevator;

import org.team7525.subsystem.SubsystemStates;

public enum ElevatorStates implements SubsystemStates {
	HIGH("HIGH", ElevatorConstants.HIGH_POSITION_HEIGHT.magnitude()),
	MID("MID", ElevatorConstants.MID_POSITION_HEIGHT.magnitude()),
	IDLE("IDLE", ElevatorConstants.IDLE_POSITION_HEIGHT.magnitude());

	ElevatorStates(String stateString, double targetHeight) {
		this.targetHeight = targetHeight;
		this.stateString = stateString;
	}

	private double targetHeight;
	private String stateString;

	public double getTargetHeight() {
		return targetHeight;
	}

	public String getStateString() {
		return stateString;
	}
}
