package frc.robot.subsystems.Vision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.GlobalConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOReal implements VisionIO {

	private PhotonCamera sideCamera;
	private PhotonCamera frontCamera;
	private PhotonPoseEstimator sideEstimator;
	private PhotonPoseEstimator frontEstimator;
	private Debouncer sideDebouncer;
	private Debouncer frontDebouncer;

	public VisionIOReal() {
		sideCamera = new PhotonCamera("Side Camera");
		frontCamera = new PhotonCamera("Front Camera");

		// Pose estimators :/
		frontEstimator = new PhotonPoseEstimator(
			GlobalConstants.Vision.APRIL_TAG_FIELD_LAYOUT,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			GlobalConstants.Vision.ROBOT_TO_FRONT_CAMERA
		);
		sideEstimator = new PhotonPoseEstimator(
			GlobalConstants.Vision.APRIL_TAG_FIELD_LAYOUT,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			GlobalConstants.Vision.ROBOT_TO_SIDE_CAMERA
		);
		sideDebouncer = new Debouncer(
			GlobalConstants.Vision.CAMERA_DEBOUNCE_TIME,
			DebounceType.kFalling
		);
		frontDebouncer = new Debouncer(
			GlobalConstants.Vision.CAMERA_DEBOUNCE_TIME,
			DebounceType.kFalling
		);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		Optional<EstimatedRobotPose> sidePose = getSidePoseEstimation();
		Optional<EstimatedRobotPose> frontPose = getFrontPoseEstimation();

		inputs.hasSideVision = sideDebouncer.calculate(sidePose.isPresent());
		inputs.hasFrontVision = frontDebouncer.calculate(frontPose.isPresent());
		inputs.sideCameraConnected = sideCamera.isConnected();
		inputs.frontCameraConnected = frontCamera.isConnected();
		inputs.sideTargetCount = sidePose.get().targetsUsed.size();
		inputs.frontTargetCount = frontPose.get().targetsUsed.size();
		if (inputs.hasSideVision) inputs.sideVisionPose = sidePose.get().estimatedPose.toPose2d();
		if (inputs.hasFrontVision) inputs.frontVisionPose = frontPose
			.get()
			.estimatedPose.toPose2d();
	}

	@Override
	public void updateRobotPose(Pose2d pose) {
		// U dont need robot pose for real life
		return;
	}

	@Override
	public void setStrategy(PoseStrategy strategy) {
		if (strategy != frontEstimator.getPrimaryStrategy()) {
			frontEstimator.setPrimaryStrategy(strategy);
			sideEstimator.setPrimaryStrategy(strategy);
		}
	}

	// Not just returning a pose3d bc timestamps needed for main pose estimation & easier to handle optional logic in vision.java
	@Override
	public Optional<EstimatedRobotPose> getSidePoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (var change : sideCamera.getAllUnreadResults()) {
			pose = sideEstimator.update(change);
		}
		return pose;
	}

	@Override
	public Optional<EstimatedRobotPose> getFrontPoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (var change : frontCamera.getAllUnreadResults()) {
			pose = frontEstimator.update(change);
		}
		return pose;
	}
}