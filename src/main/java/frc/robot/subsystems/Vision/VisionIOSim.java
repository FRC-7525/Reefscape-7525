package frc.robot.subsystems.Vision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

	private VisionSystemSim visionSim;
	private SimCameraProperties backCameraProperties;
	private SimCameraProperties frontCameraProperties;
	private PhotonCameraSim backCamera;
	private PhotonCameraSim frontCamera;
	private PhotonPoseEstimator backEstimator;
	private PhotonPoseEstimator frontEstimator;
	private Debouncer backDebouncer;
	private Debouncer frontDebouncer;
	private Pose2d robotPose;

	public VisionIOSim() {
		visionSim = new VisionSystemSim("Vision");
		backCameraProperties = new SimCameraProperties();
		frontCameraProperties = new SimCameraProperties();

		// TODO: Tune to accurate values & put in constants ig
		// A 640 x 480 camera with a 100 degree diagonal FOV.
		backCameraProperties.setCalibration(1200, 800, Rotation2d.fromDegrees(84.47));
		frontCameraProperties.setCalibration(1200, 800, Rotation2d.fromDegrees(84.47));
		// Approximate detection noise with average and standard deviation error in pixels.
		backCameraProperties.setCalibError(0.25, 0.08);
		frontCameraProperties.setCalibError(0.25, 0.08);
		// Set the camera image capture framerate (Note: this is limited by robot loop rate).
		backCameraProperties.setFPS(40);
		frontCameraProperties.setFPS(40);
		// The average and standard deviation in milliseconds of image data latency.
		backCameraProperties.setAvgLatencyMs(40);
		backCameraProperties.setLatencyStdDevMs(10);
		frontCameraProperties.setAvgLatencyMs(40);
		frontCameraProperties.setLatencyStdDevMs(10);

		backCamera = new PhotonCameraSim(new PhotonCamera("Back Camera"), backCameraProperties);
		frontCamera = new PhotonCameraSim(new PhotonCamera("Front Camera"), frontCameraProperties);

		visionSim.addAprilTags(GlobalConstants.Vision.APRIL_TAG_FIELD_LAYOUT);
		visionSim.addCamera(backCamera, GlobalConstants.Vision.ROBOT_TO_BACK_CAMERA);
		visionSim.addCamera(frontCamera, GlobalConstants.Vision.ROBOT_TO_FRONT_CAMERA);

		// Puts a camera stream onto nt4
		frontCamera.enableRawStream(true);
		frontCamera.enableProcessedStream(true);
		backCamera.enableRawStream(true);
		backCamera.enableProcessedStream(true);
		// Disable this if ur laptop is bad (makes the camera stream easy to understanda)
		frontCamera.enableDrawWireframe(true);
		backCamera.enableDrawWireframe(true);

		robotPose = new Pose2d();
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
		inputs.backCameraConnected = backCamera.getCamera().isConnected();
		inputs.frontCameraConnected = frontCamera.getCamera().isConnected();
		inputs.backTargetCount = backPose.get().targetsUsed.size();
		inputs.frontTargetCount = frontPose.get().targetsUsed.size();
		if (inputs.hasBackVision) inputs.backVisionPose = backPose.get().estimatedPose.toPose2d();
		if (inputs.hasFrontVision) inputs.frontVisionPose = frontPose
			.get()
			.estimatedPose.toPose2d();
	}

	@Override
	public void updateRobotPose(Pose2d pose) {
		robotPose = pose;
		visionSim.update(robotPose);
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
		for (var change : backCamera.getCamera().getAllUnreadResults()) {
			pose = backEstimator.update(change);
		}
		return pose;
	}

	@Override
	public Optional<EstimatedRobotPose> getFrontPoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (var change : frontCamera.getCamera().getAllUnreadResults()) {
			pose = frontEstimator.update(change);
		}
		return pose;
	}
}