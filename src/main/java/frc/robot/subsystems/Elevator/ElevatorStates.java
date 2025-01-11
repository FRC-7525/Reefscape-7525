package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Meter;

import org.team7525.subsystem.SubsystemStates;

public enum ElevatorStates implements SubsystemStates {
	L4("L4", ElevatorConstants.L4_HEIGHT.in(Meter)),
	L3("L3", ElevatorConstants.L3_HEIGHT.in(Meter)),
	L2("L2", ElevatorConstants.L2_HEIGHT.in(Meter)),
	L1("L1", ElevatorConstants.L1_HEIGHT.in(Meter)),
	CORAL_STATION("Coral Station", ElevatorConstants.L1_HEIGHT.in(Meter)),
	ALGAE_LOW("Elevator Algae Low", ElevatorConstants.ALGAE_LOW_HEIGHT.in(Meter)),
	ALGAE_HIGH("Elevator Algae High", ElevatorConstants.ALGAE_HIGH_HEIGHT.in(Meter)),
	ALGAE_PROCESSOR("Elevator Processor Level", ElevatorConstants.ALGAE_PROCESSOR_HEIGHT.in(Meter)),
	IDLE("Idle", ElevatorConstants.IDLE_HEIGHT.in(Meter));

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
