package frc.robot.Subsystems.Algaer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.team7525.subsystem.SubsystemStates;

public enum AlgaerStates implements SubsystemStates {
	INTAKING("INTAKING", AlgaerConstants.INTAKING_SPEED, AlgaerConstants.INTAKING_PIVOT),
	SHOOTING("PASSING", AlgaerConstants.PASSING_SPEED, AlgaerConstants.PASSING_PIVOT),
	IDLE("IDLE", AlgaerConstants.IDLE_SPEED, AlgaerConstants.IDLE_PIVOT);

	AlgaerStates(String stateString, AngularVelocity wheelSpeed, Angle pivotSetpoint) {
		this.wheelSpeed = wheelSpeed;
		this.pivotSetpoint = pivotSetpoint;
		this.stateString = stateString;
	}

	private AngularVelocity wheelSpeed;
	private Angle pivotSetpoint;
	private String stateString;

	public AngularVelocity getWheelSpeedSetpoint() {
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
