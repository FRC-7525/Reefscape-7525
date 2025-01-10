package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public interface VisionIO {
	@AutoLog
	public class VisionIOInputs {

		Pose2d backVisionPose;
		Pose2d frontVisionPose;
		boolean hasBackVision = false;
		boolean hasFrontVision = false;
		boolean backCameraConnected = false;
		boolean frontCameraConnected = false;
		int backTargetCount = 0;
		int frontTargetCount = 0;
	}

	public default void updateInputs(VisionIOInputs inputs) {}

	public default void updateRobotPose(Pose2d robotPose) {}

	public default void setStrategy(PoseStrategy strategy) {}

	public default Optional<EstimatedRobotPose> getFrontPoseEstimation() {
		return Optional.empty();
	}

	public default Optional<EstimatedRobotPose> getBackPoseEstimation() {
		return Optional.empty();
	}
}