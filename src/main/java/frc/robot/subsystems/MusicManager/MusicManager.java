package frc.robot.Subsystems.MusicManager;

import com.ctre.phoenix6.Orchestra;

public class MusicManager {

    private static MusicManager instance;

    private final Orchestra orchestra = new Orchestra();

    private MusicManager() {
    }

    public static MusicManager getInstance() {
        if (instance == null) {
            instance = new MusicManager();
        }
        return instance;
    }

    public void periodic() {
        
    }
}