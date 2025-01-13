package frc.robot.Subsystems.Manager;

import static frc.robot.Subsystems.Manager.ManagerConstants.*;

import frc.robot.Subsystems.Algaer.AlgaerStates;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Climber.ClimberStates;
import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("Idle", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.DOWN, AutoAlignStates.OFF),
	CLIMBING("Climbing", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.UP, AutoAlignStates.OFF),
	INTAKING_ALGAE_LOW("Intaking Algae Low", ElevatorStates.ALGAE_LOW, CoralerStates.IDLE, AlgaerStates.INTAKING, ClimberStates.DOWN, AutoAlignStates.OFF),
	INTAKING_ALGAE_HIGH("Intaking Algae High", ElevatorStates.ALGAE_HIGH, CoralerStates.IDLE, AlgaerStates.INTAKING, ClimberStates.DOWN, AutoAlignStates.OFF),
	GOING_PROCESSOR("Going to Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.GOING_TO_SHOOT, ClimberStates.DOWN, AutoAlignStates.OFF),
	SCORING_PROCESSOR("Scoring at Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.SHOOTING, ClimberStates.DOWN, AutoAlignStates.OFF),
	AUTO_ALIGN_CLOSE("Aligning Close", REEF_SCORING_LEVELS.get(Manager.getInstance().getOperatorReefScoringLevel()), CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.DOWN, REEF_TARGET_MAP.get(AAReefTarget.of(Manager.getInstance().getOperatorReefScoringLevel(), Manager.getInstance().getScoringReefLeft()))),
	AUTO_ALIGN_FAR("Aligning Close", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.DOWN, REEF_TARGET_MAP.get(AAReefTarget.of(Manager.getInstance().getOperatorReefScoringLevel(), Manager.getInstance().getScoringReefLeft()))),
	INTAKING_CORALER(
		"Intaking at Coral Station",
		ElevatorStates.IDLE,
		CoralerStates.INAKING,
		AlgaerStates.IDLE,
		ClimberStates.DOWN,
		SOURCE_TARGET_MAP.get(Manager.getInstance().getLeftSourceSelected())
	),
	SCORING_REEF_MANUAL("Scoring Reef", REEF_SCORING_LEVELS.get(Manager.getInstance().getDriverReefScoringLevel()), CoralerStates.CORALING, AlgaerStates.IDLE, ClimberStates.DOWN, AutoAlignStates.OFF),
	SCORING_REEF_AA("Scoring Reef", REEF_SCORING_LEVELS.get(Manager.getInstance().getOperatorReefScoringLevel()), CoralerStates.CORALING, AlgaerStates.IDLE, ClimberStates.DOWN, AutoAlignStates.OFF),
	TRANSITIONING_SCORING_REEF("Transitioning Scoring", REEF_SCORING_LEVELS.get(Manager.getInstance().getDriverReefScoringLevel()), CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.DOWN, AutoAlignStates.OFF);

	ManagerStates(String stateString, ElevatorStates elevatorState, CoralerStates coralerState, AlgaerStates algaerState, ClimberStates climberState, AutoAlignStates autoAlignState) {
		this.stateString = stateString;
		this.elevatorState = elevatorState;
		this.coralerState = coralerState;
		this.algaerState = algaerState;
		this.autoAlignState = autoAlignState;
		this.climberState = climberState;
	}

	private String stateString;
	private ElevatorStates elevatorState;
	private CoralerStates coralerState;
	private AlgaerStates algaerState;
	private AutoAlignStates autoAlignState;
	private ClimberStates climberState;

	@Override
	public String getStateString() {
		return stateString;
	}

	public ElevatorStates getElevatorState() {
		return elevatorState;
	}

	public CoralerStates getCoralerState() {
		return coralerState;
	}

	public AlgaerStates getAlgaerState() {
		return algaerState;
	}

	public AutoAlignStates getAutoAlignState() {
		return autoAlignState;
	}

	protected ClimberStates getClimberState() {
		return climberState;
	}
}
