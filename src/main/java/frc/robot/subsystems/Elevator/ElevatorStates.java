package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.measure.Distance;
import org.team7525.subsystem.SubsystemStates;

public enum ElevatorStates implements SubsystemStates {
	L4("L4", ElevatorConstants.L4_HEIGHT),
	L3("L3", ElevatorConstants.L3_HEIGHT),
	L2("L2", ElevatorConstants.L2_HEIGHT),
	L1("L1", ElevatorConstants.L1_HEIGHT),
	CORAL_STATION("Coral Station", ElevatorConstants.L1_HEIGHT),
	ALGAE_LOW("Elevator Algae Low", ElevatorConstants.ALGAE_LOW_HEIGHT),
	ALGAE_HIGH("Elevator Algae High", ElevatorConstants.ALGAE_HIGH_HEIGHT),
	ALGAE_PROCESSOR("Elevator Processor Level", ElevatorConstants.ALGAE_PROCESSOR_HEIGHT),
	ZEROING("Zeroing", ElevatorConstants.IDLE_HEIGHT),
	IDLE("Idle", ElevatorConstants.IDLE_HEIGHT);

	ElevatorStates(String stateString, Distance targetHeight) {
		this.targetHeight = targetHeight;
		this.stateString = stateString;
	}

	private Distance targetHeight;
	private String stateString;

	public Distance getTargetHeight() {
		return targetHeight;
	}

	public String getStateString() {
		return stateString;
	}
}
