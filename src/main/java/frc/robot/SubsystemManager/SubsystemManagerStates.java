package frc.robot.SubsystemManager;

import static frc.robot.SubsystemManager.SubsystemManagerConstants.*;

import frc.robot.SubsystemManager.SubsystemManagerConstants.AAReefTarget;
import frc.robot.Subsystems.Algaer.AlgaerStates;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Climber.ClimberStates;
import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.LED.LEDStates;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum SubsystemManagerStates implements SubsystemStates {
	IDLE("Idle", () -> ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE, ClimberStates.DOWN),
	CLIMBING("Climbing", () -> ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.CLIMBING, ClimberStates.UP),
	INTAKING_ALGAE_GROUND("Intaking Algae Ground", () -> ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.INTAKING, () -> AutoAlignStates.OFF, () -> LEDStates.ALGAE, ClimberStates.DOWN),
	INTAKING_ALGAE_LOW("Intaking Algae Low", () -> ElevatorStates.ALGAE_LOW, CoralerStates.IDLE, AlgaerStates.INTAKING, () -> AutoAlignStates.OFF, () -> LEDStates.ALGAE, ClimberStates.DOWN),
	INTAKING_ALGAE_HIGH("Intaking Algae High", () -> ElevatorStates.ALGAE_HIGH, CoralerStates.IDLE, AlgaerStates.INTAKING, () -> AutoAlignStates.OFF, () -> LEDStates.ALGAE, ClimberStates.DOWN),
	GOING_PROCESSOR("Going to Processor", () -> ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.GOING_TO_PROCESS, () -> AutoAlignStates.OFF, () -> LEDStates.ALGAE, ClimberStates.DOWN),
	SCORING_PROCESSOR("Scoring at Processor", () -> ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.PROCESSING, () -> AutoAlignStates.OFF, () -> LEDStates.ALGAE, ClimberStates.DOWN),
	HOLDING_ALGAE("Holding Algae", () -> ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.HOLDING_ALGAE, () -> AutoAlignStates.OFF, () -> LEDStates.ALGAE, ClimberStates.DOWN),
	AUTO_ALIGN_CLOSE(
		"Aligning Close",
		() -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getOperatorReefScoringLevel()),
		CoralerStates.IDLE,
		AlgaerStates.IDLE,
		() -> REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())),
		() -> LEDStates.AUTOALIGN,
		ClimberStates.DOWN
	),
	AUTO_ALIGN_FAR(
		"Aligning Far",
		() -> ElevatorStates.TRANSITIONING,
		CoralerStates.IDLE,
		AlgaerStates.IDLE,
		() -> REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())),
		() -> LEDStates.AUTOALIGN,
		ClimberStates.DOWN
	),
	INTAKING_CORALER(
		"Intaking at Coral Station",
		() -> ElevatorStates.IDLE,
		CoralerStates.INAKING,
		AlgaerStates.IDLE,
		// AutoAlignStates.OFF
		() -> SOURCE_TARGET_MAP.get(SubsystemManager.getInstance().getLeftSourceSelected()),
		() -> LEDStates.INTAKING,
		ClimberStates.DOWN
	),
	SCOING_NET("Scoring net", () -> ElevatorStates.L4, CoralerStates.IDLE, AlgaerStates.SCORING_NET, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING, ClimberStates.DOWN),
	INTAKING_CORALER_AA_OFF("Intaking Coral Station with Driver Control", () -> ElevatorStates.IDLE, CoralerStates.INAKING, AlgaerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.INTAKING, ClimberStates.DOWN),
	CENTERING_CORALER("Centering Coral", () -> ElevatorStates.IDLE, CoralerStates.CENTERING, AlgaerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.INTAKING, ClimberStates.DOWN),
	SCORING_REEF_MANUAL("Scoring Reef", () -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getDriverReefScoringLevel()), CoralerStates.CORALING, AlgaerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING, ClimberStates.DOWN),
	SCORING_REEF_AA("Scoring Reef AA", () -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getOperatorReefScoringLevel()), CoralerStates.CORALING, AlgaerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING, ClimberStates.DOWN),
	TRANSITIONING_SCORING_REEF(
		"Transitioning Scoring",
		() -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getDriverReefScoringLevel()),
		CoralerStates.IDLE,
		AlgaerStates.IDLE,
		() -> AutoAlignStates.OFF,
		() -> LED_TO_REEF_LEVEL.get(SubsystemManager.getInstance().getDriverReefScoringLevel()),
		ClimberStates.DOWN
	),
	ZEROING_ELEVATOR("Zeroing Elevator", () -> ElevatorStates.ZEROING, CoralerStates.IDLE, AlgaerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE, ClimberStates.DOWN);

	SubsystemManagerStates(String stateString, Supplier<ElevatorStates> elevatorStateSupplier, CoralerStates coralerState, AlgaerStates algaerState, Supplier<AutoAlignStates> autoAlignSupplier, Supplier<LEDStates> ledStateSupplier, ClimberStates climberState) {
		this.stateString = stateString;
		this.elevatorStateSupplier = elevatorStateSupplier;
		this.coralerState = coralerState;
		this.algaerState = algaerState;
		this.autoAlignSupplier = autoAlignSupplier;
		this.ledStateSupplier = ledStateSupplier;
		this.climberState = climberState;
	}

	private String stateString;
	private Supplier<ElevatorStates> elevatorStateSupplier;
	private CoralerStates coralerState;
	private AlgaerStates algaerState;
	private Supplier<AutoAlignStates> autoAlignSupplier;
	private Supplier<LEDStates> ledStateSupplier;
	private ClimberStates climberState;

	@Override
	public String getStateString() {
		return stateString;
	}

	public Supplier<ElevatorStates> getElevatorStateSupplier() {
		return elevatorStateSupplier;
	}

	public CoralerStates getCoralerState() {
		return coralerState;
	}

	public AlgaerStates getAlgaerState() {
		return algaerState;
	}

	public Supplier<AutoAlignStates> getAutoAlignSupplier() {
		return autoAlignSupplier;
	}

	public Supplier<LEDStates> getLedStateSupplier() {
		return ledStateSupplier;
	}

	public ClimberStates getClimberState() {
		return climberState;
	}
}
