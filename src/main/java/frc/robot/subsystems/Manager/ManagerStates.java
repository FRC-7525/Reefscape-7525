package frc.robot.Subsystems.Manager;

import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("Idle", ElevatorStates.IDLE, CoralerStates.IDLE),
	CLIMBING("Climbing", ElevatorStates.CLIMBING, CoralerStates.IDLE),
	INTAKING_CORALER("Intaking through Coraler", ElevatorStates.IDLE, CoralerStates.INAKING),
	INTAKING_ALGAE_LOW("Intaking Algae Low", ElevatorStates.ALGAE_LOW, CoralerStates.IDLE),
	INTAKING_ALGAE_HIGH("Intaking Algae High", ElevatorStates.ALGAE_HIGH, CoralerStates.IDLE),
	GOING_PROCESSOR("Going to Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE),
	SCORING_PROCESSOR("Scoring at Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE),
	AUTO_ALIGN_CLOSE("Aligning Close", ElevatorStates.IDLE, CoralerStates.IDLE),
	AUTO_ALIGN_FAR("Aligning Close", ElevatorStates.IDLE, CoralerStates.IDLE),
	SCORING_REEF("Scoring Reef", ElevatorStates.IDLE, CoralerStates.CORALING),
	TRANSITIONING_SCORING_REEF("Transitioning Scoring", ElevatorStates.IDLE, CoralerStates.IDLE),;


	


	ManagerStates(String stateString, ElevatorStates elevatorState, CoralerStates coralerState) {
		this.stateString = stateString;
		this.elevatorState = elevatorState;
		this.coralerState = coralerState;
	}

	private String stateString;
	private ElevatorStates elevatorState;
	private CoralerStates coralerState;

	@Override
	public String getStateString() {
		return stateString;
	}

	protected ElevatorStates getElevatorState() {
		return elevatorState;
	}

	protected CoralerStates getCoralerState() {
		return coralerState;
	}
}
