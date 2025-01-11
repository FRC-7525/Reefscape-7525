package frc.robot.Subsystems.Manager;

import frc.robot.Subsystems.Algaer.AlgaerStates;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("Idle", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, AutoAlignStates.OFF),
	CLIMBING("Climbing", ElevatorStates.CLIMBING, CoralerStates.IDLE, AlgaerStates.IDLE, AutoAlignStates.OFF),
	INTAKING_CORALER("Intaking at Coral Station", ElevatorStates.IDLE, CoralerStates.INAKING, AlgaerStates.IDLE, AutoAlignStates.OFF),
	INTAKING_ALGAE_LOW("Intaking Algae Low", ElevatorStates.ALGAE_LOW, CoralerStates.IDLE, AlgaerStates.INTAKING, AutoAlignStates.OFF),
	INTAKING_ALGAE_HIGH("Intaking Algae High", ElevatorStates.ALGAE_HIGH, CoralerStates.IDLE, AlgaerStates.INTAKING, AutoAlignStates.OFF),
	GOING_PROCESSOR("Going to Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.IDLE, AutoAlignStates.OFF),
	SCORING_PROCESSOR("Scoring at Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.SHOOTING, AutoAlignStates.OFF),
	AUTO_ALIGN_CLOSE("Aligning Close", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, AutoAlignStates.DRIVING_REEF),
	AUTO_ALIGN_FAR("Aligning Close", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, AutoAlignStates.DRIVING_REEF),
	SCORING_REEF("Scoring Reef", ElevatorStates.IDLE, CoralerStates.CORALING, AlgaerStates.IDLE, AutoAlignStates.OFF),
	TRANSITIONING_SCORING_REEF("Transitioning Scoring", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, AutoAlignStates.OFF),;


	// NOTE: Transitioning Scoring, AA, and any state meant to score on reef has a FILLER ELEVATOR AND POSSIBLY CORALER STATE


	ManagerStates(String stateString, ElevatorStates elevatorState, CoralerStates coralerState, AlgaerStates algaerState, AutoAlignStates autoAlignState) {
		this.stateString = stateString;
		this.elevatorState = elevatorState;
		this.coralerState = coralerState;
	}

	private String stateString;
	private ElevatorStates elevatorState;
	private CoralerStates coralerState;
	private AlgaerStates algaerState;
	private AutoAlignStates autoAlignState;

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

	protected AlgaerStates getAlgaerState() {
		return algaerState;
	}

	protected AutoAlignStates getAutoAlignState() {
		return autoAlignState;
	}
}
