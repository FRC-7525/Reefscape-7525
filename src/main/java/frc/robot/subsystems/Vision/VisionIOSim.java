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
	private SimCameraProperties sideCameraProperties;
	private SimCameraProperties frontCameraProperties;
	private PhotonCameraSim sideCamera;
	private PhotonCameraSim frontCamera;
	private PhotonPoseEstimator sideEstimator;
	private PhotonPoseEstimator frontEstimator;
	private Debouncer sideDebouncer;
	private Debouncer frontDebouncer;
	private Pose2d robotPose;

	public VisionIOSim() {
		visionSim = new VisionSystemSim("Vision");
		sideCameraProperties = new SimCameraProperties();
		frontCameraProperties = new SimCameraProperties();

		// TODO: Tune to accurate values & put in constants ig
		// A 640 x 480 camera with a 100 degree diagonal FOV.
		sideCameraProperties.setCalibration(1200, 800, Rotation2d.fromDegrees(84.47));
		frontCameraProperties.setCalibration(1200, 800, Rotation2d.fromDegrees(84.47));
		// Approximate detection noise with average and standard deviation error in pixels.
		sideCameraProperties.setCalibError(0.25, 0.08);
		frontCameraProperties.setCalibError(0.25, 0.08);
		// Set the camera image capture framerate (Note: this is limited by robot loop rate).
		sideCameraProperties.setFPS(40);
		frontCameraProperties.setFPS(40);
		// The average and standard deviation in milliseconds of image data latency.
		sideCameraProperties.setAvgLatencyMs(40);
		sideCameraProperties.setLatencyStdDevMs(10);
		frontCameraProperties.setAvgLatencyMs(40);
		frontCameraProperties.setLatencyStdDevMs(10);

		sideCamera = new PhotonCameraSim(new PhotonCamera("Side Camera"), sideCameraProperties);
		frontCamera = new PhotonCameraSim(new PhotonCamera("Front Camera"), frontCameraProperties);

		visionSim.addAprilTags(GlobalConstants.Vision.APRIL_TAG_FIELD_LAYOUT);
		visionSim.addCamera(sideCamera, GlobalConstants.Vision.ROBOT_TO_SIDE_CAMERA);
		visionSim.addCamera(frontCamera, GlobalConstants.Vision.ROBOT_TO_FRONT_CAMERA);

		// Puts a camera stream onto nt4
		frontCamera.enableRawStream(true);
		frontCamera.enableProcessedStream(true);
		sideCamera.enableRawStream(true);
		sideCamera.enableProcessedStream(true);
		// Disable this if ur laptop is bad (makes the camera stream easy to understanda)
		frontCamera.enableDrawWireframe(true);
		sideCamera.enableDrawWireframe(true);

		robotPose = new Pose2d();
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
		inputs.sideCameraConnected = sideCamera.getCamera().isConnected();
		inputs.frontCameraConnected = frontCamera.getCamera().isConnected();
		inputs.sideTargetCount = sidePose.get().targetsUsed.size();
		inputs.frontTargetCount = frontPose.get().targetsUsed.size();
		if (inputs.hasSideVision) inputs.sideVisionPose = sidePose.get().estimatedPose.toPose2d();
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
			sideEstimator.setPrimaryStrategy(strategy);
		}
	}

	// Not just returning a pose3d bc timestamps needed for main pose estimation & easier to handle optional logic in vision.java
	@Override
	public Optional<EstimatedRobotPose> getSidePoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (var change : sideCamera.getCamera().getAllUnreadResults()) {
			pose = sideEstimator.update(change);
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