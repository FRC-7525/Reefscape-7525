package frc.robot.Subsystems.Climber;

import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.units.measure.Distance;

public enum ClimberStates implements SubsystemStates {
	UP("UP", ClimberConstants.UP),
	DOWN("DOWN", ClimberConstants.DOWN),;

	ClimberStates(String stateString, Distance targetHeight) {
		this.targetHeight = targetHeight;
		this.stateString = stateString;
	}

	private Distance targetHeight;
	private String stateString;

	public Distance getTargetHeight() {
		return targetHeight;
	}

	public String getStateString() {
		return stateString;
	}
}
