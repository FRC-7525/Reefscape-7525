package frc.robot.Subsystems.Manager;

import static frc.robot.Subsystems.Manager.ManagerConstants.*;

import frc.robot.Subsystems.Algaer.AlgaerStates;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Climber.ClimberStates;
import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.LED.LEDStates;
import frc.robot.Subsystems.Manager.ManagerConstants.AAReefTarget;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("Idle", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.IDLE, AutoAlignStates.OFF),
	CLIMBING("Climbing", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.IDLE, AutoAlignStates.OFF),
	INTAKING_ALGAE_LOW("Intaking Algae Low", ElevatorStates.ALGAE_LOW, CoralerStates.IDLE, AlgaerStates.INTAKING, ClimberStates.IDLE, AutoAlignStates.OFF),
	INTAKING_ALGAE_HIGH("Intaking Algae High", ElevatorStates.ALGAE_HIGH, CoralerStates.IDLE, AlgaerStates.INTAKING, ClimberStates.IDLE, AutoAlignStates.OFF),
	GOING_PROCESSOR("Going to Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.GOING_TO_SHOOT, ClimberStates.IDLE, AutoAlignStates.OFF),
	SCORING_PROCESSOR("Scoring at Processor", ElevatorStates.ALGAE_PROCESSOR, CoralerStates.IDLE, AlgaerStates.SHOOTING, ClimberStates.IDLE, AutoAlignStates.OFF),
	AUTO_ALIGN_CLOSE(
		"Aligning Close",
		REEF_SCORING_LEVELS.get(Manager.getInstance().getOperatorReefScoringLevel()),
		CoralerStates.IDLE,
		AlgaerStates.IDLE,
		ClimberStates.IDLE,
		REEF_TARGET_MAP.get(AAReefTarget.of(Manager.getInstance().getOperatorReefScoringLevel(), Manager.getInstance().getScoringReefLeft()))
	),
	AUTO_ALIGN_FAR("Aligning Close", ElevatorStates.IDLE, CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.IDLE, REEF_TARGET_MAP.get(AAReefTarget.of(Manager.getInstance().getOperatorReefScoringLevel(), Manager.getInstance().getScoringReefLeft()))),
	INTAKING_CORALER(
		"Intaking at Coral Station",
		() -> ElevatorStates.IDLE,
		CoralerStates.INAKING,
		AlgaerStates.IDLE,
		ClimberStates.IDLE,
		// AutoAlignStates.OFF
		() -> SOURCE_TARGET_MAP.get(Manager.getInstance().getLeftSourceSelected()),
		LEDStates.ORANGE
	),
	SCORING_REEF_MANUAL("Scoring Reef", REEF_SCORING_LEVELS.get(Manager.getInstance().getDriverReefScoringLevel()), CoralerStates.CORALING, AlgaerStates.IDLE, ClimberStates.IDLE, AutoAlignStates.OFF),
	SCORING_REEF_AA("Scoring Reef", REEF_SCORING_LEVELS.get(Manager.getInstance().getOperatorReefScoringLevel()), CoralerStates.CORALING, AlgaerStates.IDLE, ClimberStates.IDLE, AutoAlignStates.OFF),
	TRANSITIONING_SCORING_REEF("Transitioning Scoring", REEF_SCORING_LEVELS.get(Manager.getInstance().getDriverReefScoringLevel()), CoralerStates.IDLE, AlgaerStates.IDLE, ClimberStates.IDLE, AutoAlignStates.OFF);

	ManagerStates(String stateString, Supplier<ElevatorStates> elevatorStateSupplier, CoralerStates coralerState, AlgaerStates algaerState, Supplier<AutoAlignStates> autoAlignSupplier, LEDStates ledState) {
		this.stateString = stateString;
		this.elevatorStateSupplier = elevatorStateSupplier;
		this.coralerState = coralerState;
		this.algaerState = algaerState;
		this.climberState = climberState;
		this.autoAlignState = autoAlignState;
		this.autoAlignSupplier = autoAlignSupplier;
		this.ledState = ledState;
	}

	private String stateString;
	private Supplier<ElevatorStates> elevatorStateSupplier;
	private CoralerStates coralerState;
	private AlgaerStates algaerState;
	private AutoAlignStates autoAlignState;
	private ClimberStates climberState;
	private Supplier<AutoAlignStates> autoAlignSupplier;
	private LEDStates ledState;

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

	public ClimberStates getClimberState() {
		return climberState;
	}

	public AutoAlignStates getAutoAlignState() {
		return autoAlignState;
	}

	public Supplier<AutoAlignStates> getAutoAlignSupplier() {
		return autoAlignSupplier;
	}

	public LEDStates getLedState() {
		return ledState;
	}
}
