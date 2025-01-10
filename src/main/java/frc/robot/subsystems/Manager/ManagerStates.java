package frc.robot.Subsystems.Manager;

import org.team7525.subsystem.SubsystemStates;

import frc.robot.Subsystems.Coraler.CoralerStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public enum ManagerStates implements SubsystemStates{
    IDLE("Idle", ElevatorStates.IDLE, CoralerStates.IDLE),
    L1_TRANSITION("Getting To L1", ElevatorStates.L1, CoralerStates.IDLE),
    L1_SCORING("Scoring L1", ElevatorStates.L1, CoralerStates.CORALING),
    L2_TRANSITION("Getting To L2", ElevatorStates.L2, CoralerStates.IDLE),
    L2_SCORING("Scoring L2", ElevatorStates.L2, CoralerStates.CORALING),
    L3_TRANSITION("Getting To L3", ElevatorStates.L3, CoralerStates.IDLE),
    L3_SCORING("Scoring L3", ElevatorStates.L3, CoralerStates.CORALING),
    L4_TRANSITION("Getting To L4", ElevatorStates.L4, CoralerStates.IDLE),
    L4_SCORING("Scoring L4", ElevatorStates.L4, CoralerStates.CORALING),
    CORAL_STATION_TRANSITION("Getting To Coral Station", ElevatorStates.CORAL_STATION, CoralerStates.IDLE),
    CORAL_STATION_INTAKING("Scoring Coral Station", ElevatorStates.CORAL_STATION, CoralerStates.CORALING);


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
