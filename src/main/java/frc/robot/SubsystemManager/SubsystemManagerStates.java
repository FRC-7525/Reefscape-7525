package frc.robot.SubsystemManager;

import static frc.robot.SubsystemManager.SubsystemManagerConstants.*;

import frc.robot.SubsystemManager.SubsystemManagerConstants.AAReefTarget;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.LED.LEDStates;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum SubsystemManagerStates implements SubsystemStates {
	IDLE("Idle", () -> ElevatorStates.IDLE, CoralerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE),
	AUTO_ALIGN_CLOSE(
		"Aligning Close",
		() -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getOperatorReefScoringLevel()),
		CoralerStates.IDLE,
		() -> REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())),
		() -> LEDStates.AUTOALIGN
	),
	AUTO_ALIGN_FAR(
		"Aligning Far",
		() -> SubsystemManager.getInstance().getOperatorReefScoringLevel() < 3 ? ElevatorStates.IDLE : ElevatorStates.TRANSITIONING,
		CoralerStates.IDLE,
		() -> REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())),
		() -> LEDStates.AUTOALIGN
	),
	INTAKING_CORALER(
		"Intaking at Coral Station",
		() -> ElevatorStates.IDLE,
		CoralerStates.INAKING,
		// AutoAlignStates.OFF
		() -> SOURCE_TARGET_MAP.get(SubsystemManager.getInstance().getLeftSourceSelected()),
		() -> LEDStates.INTAKING
	),
	INTAKING_CORALER_AA_OFF("Intaking Coral Station with Driver Control", () -> ElevatorStates.IDLE, CoralerStates.INAKING, () -> AutoAlignStates.OFF, () -> LEDStates.INTAKING),
	CENTERING_CORALER("Centering Coral", () -> ElevatorStates.IDLE, CoralerStates.CENTERING, () -> AutoAlignStates.OFF, () -> LEDStates.INTAKING),
	SCORING_REEF_MANUAL("Scoring Reef", () -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getDriverReefScoringLevel()), CoralerStates.CORALING, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING),
	SCORING_REEF_AA("Scoring Reef AA", () -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getOperatorReefScoringLevel()), CoralerStates.CORALING, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING),
	TRANSITIONING_SCORING_REEF("Transitioning Scoring", () -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getDriverReefScoringLevel()), CoralerStates.IDLE, () -> AutoAlignStates.OFF, () -> LED_TO_REEF_LEVEL.get(SubsystemManager.getInstance().getDriverReefScoringLevel())),
	ZEROING_ELEVATOR("Zeroing Elevator", () -> ElevatorStates.ZEROING, CoralerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE);

	SubsystemManagerStates(String stateString, Supplier<ElevatorStates> elevatorStateSupplier, CoralerStates coralerState, Supplier<AutoAlignStates> autoAlignSupplier, Supplier<LEDStates> ledStateSupplier) {
		this.stateString = stateString;
		this.elevatorStateSupplier = elevatorStateSupplier;
		this.coralerState = coralerState;
		this.autoAlignSupplier = autoAlignSupplier;
		this.ledStateSupplier = ledStateSupplier;
	}

	private String stateString;
	private Supplier<ElevatorStates> elevatorStateSupplier;
	private CoralerStates coralerState;
	private Supplier<AutoAlignStates> autoAlignSupplier;
	private Supplier<LEDStates> ledStateSupplier;

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

	public Supplier<AutoAlignStates> getAutoAlignSupplier() {
		return autoAlignSupplier;
	}

	public Supplier<LEDStates> getLedStateSupplier() {
		return ledStateSupplier;
	}
}
