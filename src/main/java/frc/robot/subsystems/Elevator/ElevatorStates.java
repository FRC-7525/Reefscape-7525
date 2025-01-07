package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Meter;

import org.team7525.subsystem.SubsystemStates;

public enum ElevatorStates implements SubsystemStates {
	HIGH("HIGH", ElevatorConstants.HIGH_POSITION_HEIGHT.in(Meter)),
	MID("MID", ElevatorConstants.MID_POSITION_HEIGHT.in(Meter)),
	DOWN("Down", ElevatorConstants.DOWN_POSITION_HEIGHT.in(Meter));

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
