package frc.robot.Subsystems.ObstacleVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface ObstacleVisionIO {
    @AutoLog
	public static class VisionIOInputs {
		public boolean connected = false;
	}

    public ArrayList<Pose2d> getObstaclePoses();
    
    public void updateInputs(VisionIOInputs inputs);
}
