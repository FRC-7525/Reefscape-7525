package frc.robot.Subsystems.Algaer;

import edu.wpi.first.units.measure.Angle;
import org.team7525.subsystem.SubsystemStates;

public enum AlgaerStates implements SubsystemStates {
	INTAKING("INTAKING", AlgaerConstants.INTAKING_SPEED, AlgaerConstants.INTAKING_PIVOT),
	PROCESSING("PASSING", AlgaerConstants.PROCESSING_SPEED, AlgaerConstants.PROCESSING_PIVOT),
	GOING_TO_PROCESS("GOING TO SHOOT", AlgaerConstants.IDLE_SPEED, AlgaerConstants.PROCESSING_PIVOT),
	HOLDING_ALGAE("Holding Algae", AlgaerConstants.IDLE_SPEED, AlgaerConstants.HOLDING_PIVOT),
	IDLE("IDLE", AlgaerConstants.IDLE_SPEED, AlgaerConstants.IDLE_PIVOT);

	AlgaerStates(String stateString, double wheelSpeed, Angle pivotSetpoint) {
		this.wheelSpeed = wheelSpeed;
		this.pivotSetpoint = pivotSetpoint;
		this.stateString = stateString;
	}

	private double wheelSpeed;
	private Angle pivotSetpoint;
	private String stateString;

	public double getWheelSpeedSetpoint() {
		return wheelSpeed;
	}

	public Angle getPivotSetpoint() {
		return pivotSetpoint;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
