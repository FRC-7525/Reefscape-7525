package frc.robot.Subsystems.MusicManager;

import com.ctre.phoenix6.Orchestra;
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

	private MusicManager() {
        playMusic.setDefaultOption(SUBSYSTEM_NAME + "/Music Off", false);
        playMusic.addOption(SUBSYSTEM_NAME + "/Music On", true);

        songToPlay.setDefaultOption("Thick of It", "song1.chrp");
        songToPlay.addOption("???", "song2.chrp");
    }

	public static MusicManager getInstance() {
		if (instance == null) {
			instance = new MusicManager();
		}
		return instance;
	}

	public void playMusic() {
        if (!orchestra.isPlaying()) {
            orchestra.loadMusic(songToPlay.getSelected());
        }
        orchestra.play();
    }

	public void addMotor(TalonFX motor) {
		orchestra.addInstrument(motor);
	}

	public void removeAllMotors() {
		orchestra.clearInstruments();
	}

	public void addAllSubsystemInstruments() {
        orchestra.addInstrument(Algaer.getInstance().getWheelMotor());
        orchestra.addInstrument(Algaer.getInstance().getPivotMotor());
        orchestra.addInstrument(Coraler.getInstance().getMotor());
        orchestra.addInstrument(Elevator.getInstance().getLeftMotor());
        orchestra.addInstrument(Elevator.getInstance().getRightMotor());
        for (TalonFX motor : Drive.getInstance().getDriveMotors()) {
            orchestra.addInstrument(motor);
        }
        for (TalonFX motor : Drive.getInstance().getSteerMotors()) {
            orchestra.addInstrument(motor);
        }
    }

    public Boolean playMusicEnabled() {
        return playMusic.getSelected();
    }

    public void stopMusic() {
        orchestra.stop();
    }
}
