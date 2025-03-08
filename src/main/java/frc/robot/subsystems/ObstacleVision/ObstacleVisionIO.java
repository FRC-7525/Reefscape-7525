package frc.robot.Subsystems.ObstacleVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Subsystems.ObstacleVision.ObstacleVisionConstants.ObstaclePoseObservation;

public interface ObstacleVisionIO {
    @AutoLog
	public static class VisionIOInputs {
		public boolean connected = false;
	}

    public ArrayList<ObstaclePoseObservation> getObstaclePoses();
    
    public void updateInputs(VisionIOInputs inputs);
}
