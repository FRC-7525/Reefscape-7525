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

	private PhotonCamera backCamera;
	private PhotonCamera frontCamera;
	private PhotonPoseEstimator backEstimator;
	private PhotonPoseEstimator frontEstimator;
	private Debouncer backDebouncer;
	private Debouncer frontDebouncer;

	public VisionIOReal() {
		backCamera = new PhotonCamera("Back Camera");
		frontCamera = new PhotonCamera("Front Camera");

		// Pose estimators :/
		frontEstimator = new PhotonPoseEstimator(
			GlobalConstants.Vision.APRIL_TAG_FIELD_LAYOUT,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			GlobalConstants.Vision.ROBOT_TO_FRONT_CAMERA
		);
		backEstimator = new PhotonPoseEstimator(
			GlobalConstants.Vision.APRIL_TAG_FIELD_LAYOUT,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			GlobalConstants.Vision.ROBOT_TO_BACK_CAMERA
		);
		backDebouncer = new Debouncer(
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
		Optional<EstimatedRobotPose> backPose = getBackPoseEstimation();
		Optional<EstimatedRobotPose> frontPose = getFrontPoseEstimation();

		inputs.hasBackVision = backDebouncer.calculate(backPose.isPresent());
		inputs.hasFrontVision = frontDebouncer.calculate(frontPose.isPresent());
		inputs.backCameraConnected = backCamera.isConnected();
		inputs.frontCameraConnected = frontCamera.isConnected();
		inputs.backTargetCount = backPose.get().targetsUsed.size();
		inputs.frontTargetCount = frontPose.get().targetsUsed.size();
		if (inputs.hasBackVision) inputs.backVisionPose = backPose.get().estimatedPose.toPose2d();
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
			backEstimator.setPrimaryStrategy(strategy);
		}
	}

	// Not just returning a pose3d bc timestamps needed for main pose estimation & easier to handle optional logic in vision.java
	@Override
	public Optional<EstimatedRobotPose> getBackPoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (var change : backCamera.getAllUnreadResults()) {
			pose = backEstimator.update(change);
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
