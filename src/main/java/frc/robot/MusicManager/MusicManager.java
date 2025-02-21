package frc.robot.MusicManager;

import org.team7525.misc.Tracer;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Subsystems.Algaer.Algaer;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;

public class MusicManager {

	private static MusicManager instance;

	private final Orchestra orchestra = new Orchestra();
	private final SendableChooser<Boolean> playMusic = new SendableChooser<>();
	private final SendableChooser<String> songToPlay = new SendableChooser<>();

	private final String SUBSYSTEM_NAME = "MusicManager";
	private final String MUSIC_DIR = "music";

	private final AudioConfigs audioConfigs = new AudioConfigs();

	private Boolean hasInstruments;

	private MusicManager() {
		playMusic.setDefaultOption(SUBSYSTEM_NAME + "/Music Off", false);
		playMusic.addOption(SUBSYSTEM_NAME + "/Music On", true);

		songToPlay.setDefaultOption("Lalal", MUSIC_DIR + "/output.chrp");
		hasInstruments = false;

		audioConfigs.AllowMusicDurDisable = true;
	}

	public static MusicManager getInstance() {
		if (instance == null) {
			instance = new MusicManager();
		}
		return instance;
	}

	public void playMusic() {
		Tracer.startTrace(SUBSYSTEM_NAME);
		if (!orchestra.isPlaying()) {
			orchestra.loadMusic(songToPlay.getSelected());
		}
		orchestra.play();
		Tracer.endTrace();
	}

	public void addMotor(TalonFX motor) {
		orchestra.addInstrument(motor);
		motor.getConfigurator().apply(audioConfigs);
	}

	public void removeAllMotors() {
		hasInstruments = false;
		orchestra.clearInstruments();
	}

	public void addAllSubsystemInstruments() {
		hasInstruments = true;
		orchestra.addInstrument(Algaer.getInstance().getWheelMotor());
		Algaer.getInstance().getWheelMotor().getConfigurator().apply(audioConfigs);
		orchestra.addInstrument(Coraler.getInstance().getMotor());
		Coraler.getInstance().getMotor().getConfigurator().apply(audioConfigs);
		orchestra.addInstrument(Elevator.getInstance().getLeftMotor());
		Elevator.getInstance().getLeftMotor().getConfigurator().apply(audioConfigs);
		orchestra.addInstrument(Elevator.getInstance().getRightMotor());
		Elevator.getInstance().getRightMotor().getConfigurator().apply(audioConfigs);

		for (TalonFX motor : Drive.getInstance().getDriveMotors()) {
			orchestra.addInstrument(motor);
			motor.getConfigurator().apply(audioConfigs);
		}
		for (TalonFX motor : Drive.getInstance().getSteerMotors()) {
			orchestra.addInstrument(motor);
			motor.getConfigurator().apply(audioConfigs);
		}
	}

	public Boolean playMusicEnabled() {
		return playMusic.getSelected();
	}

	public void stopMusic() {
		orchestra.stop();
	}

	public boolean hasInstruments() {
		return hasInstruments;
	}
}
