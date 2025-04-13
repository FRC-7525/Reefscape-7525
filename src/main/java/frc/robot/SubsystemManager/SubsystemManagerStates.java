package frc.robot.SubsystemManager;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.SubsystemManager.SubsystemManagerConstants.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.L3_HEIGHT;

import frc.robot.SubsystemManager.SubsystemManagerConstants.AAReefTarget;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.LED.LEDStates;
import frc.robot.Subsystems.Passthrough.PassthroughStates;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum SubsystemManagerStates implements SubsystemStates {
	IDLE("Idle", () -> ElevatorStates.IDLE, CoralerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE, PassthroughStates.OFF),
	AUTO_ALIGN_CLOSE(
		"Aligning Close",
		() -> REEF_SCORING_LEVELS.get(SubsystemManager.getInstance().getOperatorReefScoringLevel()),
		CoralerStates.IDLE,
		() -> {
			return SubsystemManager.getInstance().getOperatorReefScoringLevel() != 1 ? 
				REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())) : 
				L1_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft()));
		},
		() -> LEDStates.AUTOALIGN,
		PassthroughStates.OFF
	),
	AUTO_ALIGN_FAR(
		"Aligning Far",
		() -> SubsystemManager.getInstance().getOperatorReefScoringLevel() < 3 ? ElevatorStates.IDLE : ElevatorStates.TRANSITIONING,
		CoralerStates.IDLE,
		() -> {
			return SubsystemManager.getInstance().getOperatorReefScoringLevel() != 1 ? 
				REEF_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft())) : 
				L1_TARGET_MAP.get(AAReefTarget.of(SubsystemManager.getInstance().getHexagonTargetSide(), SubsystemManager.getInstance().getScoringReefLeft()));
		},
		() -> LEDStates.AUTOALIGN,
		PassthroughStates.OFF
	),
	INTAKING_CORALER(
		"Intaking at Coral Station",
		() -> ElevatorStates.IDLE,
		CoralerStates.INAKING,
		() -> (Elevator.getInstance().getHeight().in(Meters) < L3_HEIGHT.in(Meters)) ? SOURCE_TARGET_MAP.get(SubsystemManager.getInstance().getLeftSourceSelected()) : AutoAlignStates.OFF,
		() -> LEDStates.INTAKING,
		PassthroughStates.INTAKING
	),
	INTAKING_CORALER_AA_OFF("Intaking Coral Station with Driver Control", () -> ElevatorStates.IDLE, CoralerStates.INAKING, () -> AutoAlignStates.OFF, () -> LEDStates.INTAKING, PassthroughStates.INTAKING),
	OUTTAKING("Outtaking coral", () -> ElevatorStates.IDLE, CoralerStates.OUTTAKING, () -> AutoAlignStates.OFF, () -> LEDStates.INTAKING, PassthroughStates.INTAKING),
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
	ZEROING_ELEVATOR("Zeroing Elevator", () -> ElevatorStates.ZEROING, CoralerStates.IDLE, () -> AutoAlignStates.OFF, () -> LEDStates.IDLE, PassthroughStates.OFF),
	SCORING_L1("Scoring L1", () -> ElevatorStates.L1_SCORING, CoralerStates.SCORING_L1, () -> AutoAlignStates.OFF, () -> LEDStates.SCORING, PassthroughStates.OFF);

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
