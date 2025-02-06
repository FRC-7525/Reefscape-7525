package frc.robot.Subsystems.MusicManager;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

public class MusicManager {

	private static MusicManager instance;

	private final Orchestra orchestra = new Orchestra();

	private MusicManager() {}

	public static MusicManager getInstance() {
		if (instance == null) {
			instance = new MusicManager();
		}
		return instance;
	}

	public void periodic() {}

	private void addMotor(TalonFX motor) {
		orchestra.addInstrument(motor);
	}

	public void removeAllMotors() {
		orchestra.clearInstruments();
	}

	public void addAllSubsystemInstruments() {}
}
