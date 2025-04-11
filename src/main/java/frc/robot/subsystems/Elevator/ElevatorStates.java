package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.measure.Distance;
import org.team7525.subsystem.SubsystemStates;

public enum ElevatorStates implements SubsystemStates {
	TRANSITIONING("Transitioning Elevator Level", ElevatorConstants.TRANSITION_HEIGHT),
	L4("L4", ElevatorConstants.L4_HEIGHT),
	L3("L3", ElevatorConstants.L3_HEIGHT),
	L2("L2", ElevatorConstants.L2_HEIGHT),
	L1("L1", ElevatorConstants.L1_HEIGHT),
	CORAL_STATION("Coral Station", ElevatorConstants.L1_HEIGHT),
	ZEROING("Zeroing", ElevatorConstants.IDLE_HEIGHT),
	IDLE("Idle", ElevatorConstants.IDLE_HEIGHT),
	L1_SCORING("L1 Scoring", ElevatorConstants.L1_SCORING_HEIGHT);

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
