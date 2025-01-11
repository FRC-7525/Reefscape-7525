package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.units.measure.Distance;
import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	UP("UP", ClimberConstants.UP),
	DOWN("DOWN", ClimberConstants.DOWN),
	IDLE("IDLE", ClimberConstants.IDLE);

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
