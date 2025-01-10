package frc.robot.Subsystems.Manager;

import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("Idle", ElevatorStates.DOWN, CoralerStates.IDLE);

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
