package frc.robot.SubsystemManager;

import static frc.robot.SubsystemManager.SubsystemManagerConstants.*;

import frc.robot.SubsystemManager.SubsystemManagerConstants.AAReefTarget;
import frc.robot.subsystems.AutoAlign.AutoAlignStates;
import frc.robot.subsystems.Coraler.CoralerStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.LED.LEDStates;
import frc.robot.subsystems.Passthrough.PassthroughStates;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum SubsystemManagerStates implements SubsystemStates {
	IDLE("Idle", () -> ElevatorStates.IDLE, CoralerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE, PassthroughStates.OFF),
	AUTO_ALIGN_CLOSE(
		"Aligning Close",
		() -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getOperatorReefScoringLevel()),
		CoralerStates.IDLE,
		() -> REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())),
		() -> LEDStates.AUTOALIGN,
		PassthroughStates.OFF
	),
	AUTO_ALIGN_FAR(
		"Aligning Far",
		() -> SubsystemManager.getInstance().getOperatorReefScoringLevel() < 3 ? ElevatorStates.IDLE : ElevatorStates.TRANSITIONING,
		CoralerStates.IDLE,
		() -> REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())),
		() -> LEDStates.AUTOALIGN,
		PassthroughStates.OFF
	),
	INTAKING_CORALER("Intaking at Coral Station", () -> ElevatorStates.IDLE, CoralerStates.INAKING, () -> SOURCE_TARGET_MAP.get(SubsystemManager.getInstance().getLeftSourceSelected()), () -> LEDStates.INTAKING, PassthroughStates.INTAKING),
	INTAKING_CORALER_AA_OFF("Intaking Coral Station with Driver Control", () -> ElevatorStates.IDLE, CoralerStates.INAKING, () -> AutoAlignStates.OFF, () -> LEDStates.INTAKING, PassthroughStates.INTAKING),
	SCORING_REEF_MANUAL("Scoring Reef", () -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getDriverReefScoringLevel()), CoralerStates.CORALING, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING, PassthroughStates.OFF),
	SCORING_REEF_AA("Scoring Reef AA", () -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getOperatorReefScoringLevel()), CoralerStates.CORALING, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING, PassthroughStates.OFF),
	TRANSITIONING_SCORING_REEF(
		"Transitioning Scoring",
		() -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getDriverReefScoringLevel()),
		CoralerStates.IDLE,
		() -> AutoAlignStates.OFF,
		() -> LED_TO_REEF_LEVEL.get(SubsystemManager.getInstance().getDriverReefScoringLevel()),
		PassthroughStates.OFF
	),
	ZEROING_ELEVATOR("Zeroing Elevator", () -> ElevatorStates.ZEROING, CoralerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE, PassthroughStates.OFF);

	SubsystemManagerStates(String stateString, Supplier<ElevatorStates> elevatorStateSupplier, CoralerStates coralerState, Supplier<AutoAlignStates> autoAlignSupplier, Supplier<LEDStates> ledStateSupplier, PassthroughStates passthroughState) {
		this.stateString = stateString;
		this.elevatorStateSupplier = elevatorStateSupplier;
		this.coralerState = coralerState;
		this.autoAlignSupplier = autoAlignSupplier;
		this.ledStateSupplier = ledStateSupplier;
		this.passthroughState = passthroughState;
	}

	private String stateString;
	private Supplier<ElevatorStates> elevatorStateSupplier;
	private CoralerStates coralerState;
	private Supplier<AutoAlignStates> autoAlignSupplier;
	private Supplier<LEDStates> ledStateSupplier;
	private PassthroughStates passthroughState;

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

	public PassthroughStates getPassthroughState() {
		return passthroughState;
	}
}
