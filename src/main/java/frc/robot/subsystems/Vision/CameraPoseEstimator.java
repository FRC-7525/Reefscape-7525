package frc.robot.Subsystems.Vision;

import static frc.robot.Subsystems.Vision.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.AbstractMap.SimpleEntry;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team7525.misc.VisionUtil.CameraResolution;

/**
 * A class containing a {@link PhotonCamera} and a {@link PhotonPoseEstimator}
 * to keep the two
 * objects together.
 */
public class CameraPoseEstimator {

	public final PhotonCamera camera;
	public final Transform3d robotToCamera;
	public final CameraResolution resolution;

	public final PhotonPoseEstimator poseEstimator;

	/**
	 * A class containing a {@link PhotonCamera} and a {@link PhotonPoseEstimator}
	 * to keep the two
	 * objects together.
	 *
	 * @param name          The name of the camera.
	 * @param robotToCamera The offset from the center of the robot to the camera.
	 * @param poseStrategy  The {@link PoseStrategy} to use for the
	 *                      {@link PhotonPoseEstimator}.
	 */
	public CameraPoseEstimator(PhotonCamera camera, Transform3d robotToCamera, PhotonPoseEstimator.PoseStrategy poseStrategy, CameraResolution resolution) {
		this.camera = camera;
		this.robotToCamera = robotToCamera;
		this.resolution = resolution;

		this.poseEstimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, poseStrategy, robotToCamera);
	}

	/**
	 * Updates the {@link PhotonPoseEstimator} by retrieving the unread results from
	 * the camera and
	 * estimating the robot's pose based on the latest result.
	 *
	 * @return A {@link SimpleEntry} containing: <br>
	 *         <br>
	 *         - An {@link Optional} of {@link EstimatedRobotPose} if the pose
	 *         estimation is successful,
	 *         otherwise an empty {@link Optional}. (Accessible via
	 *         {@link SimpleEntry}.getKey()) <br>
	 *         </br>
	 *         - An {@link Optional} of {@link PhotonPipelineResult} containing the
	 *         latest result
	 *         from the camera, or an empty {@link Optional} if there are no unread
	 *         results. (Accessible
	 *         via {@link SimpleEntry}.getValue())
	 */
	public SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>> update() {
		List<PhotonPipelineResult> results = camera.getAllUnreadResults();

		Optional<PhotonPipelineResult> result = results.isEmpty() ? Optional.empty() : Optional.of(results.get(results.size() - 1)); // TODO Make sure this is the latest result, rather than oldest

		return new SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>>(result.flatMap(poseEstimator::update), result);
	}
}
