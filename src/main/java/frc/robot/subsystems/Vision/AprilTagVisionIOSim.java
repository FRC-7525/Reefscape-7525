package frc.robot.Subsystems.Vision;

import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Vision.VisionConstants.CameraResolution;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagVisionIOSim implements AprilTagVisionIO {

	private final VisionSystemSim visionSim;

	// Camera simulators
	private PhotonCameraSim frontRightSim;
	private PhotonCameraSim backLeftSim;

	// Pose estimators
	private CameraPoseEstimator frontRightPose;
	private CameraPoseEstimator backLeftPose;

	private CameraPoseEstimator[] poseEstimators;

	private Pose3d[] poseArray = new Pose3d[4];
	private double[] timestampArray = new double[4];
	private double[] visionStdArray = new double[4 * 3];
	private double[] latencyArray;

	public AprilTagVisionIOSim() {
		PhotonCamera frontLeft = new PhotonCamera(FRONT_CAMERA_NAME);
		PhotonCamera backRight = new PhotonCamera(BACK_CAMERA_NAME);

		frontRightPose = new CameraPoseEstimator(frontLeft, ROBOT_TO_FRONT_CAMERA, POSE_STRATEGY, CameraResolution.HIGH_RES);
		backLeftPose = new CameraPoseEstimator(backRight, ROBOT_TO_BACK_CAMERA, POSE_STRATEGY, CameraResolution.HIGH_RES);

		this.poseEstimators = new CameraPoseEstimator[] { frontRightPose, backLeftPose };

		visionSim = new VisionSystemSim("main");
		visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);

		SimCameraProperties cameraProps = new SimCameraProperties(); // Corresponds to high-res cameras
		cameraProps.setCalibration(1600, 1200, Rotation2d.fromDegrees(75));
		cameraProps.setCalibError(0.25, 0.10);
		cameraProps.setFPS(15);
		cameraProps.setAvgLatencyMs(50);
		cameraProps.setLatencyStdDevMs(15);

		frontRightSim = new PhotonCameraSim(frontLeft, cameraProps);
		backLeftSim = new PhotonCameraSim(backRight, cameraProps);

		visionSim.addCamera(frontRightSim, ROBOT_TO_FRONT_CAMERA);
		visionSim.addCamera(backLeftSim, ROBOT_TO_BACK_CAMERA);

		frontRightSim.enableDrawWireframe(true);
		backLeftSim.enableDrawWireframe(true);
	}

	@Override
	public void updateInputs(AprilTagVisionIOInputs inputs) {
		getEstimatedPoseUpdates(poseEstimators, poseArray, timestampArray, visionStdArray, latencyArray);
		inputs.visionPoses = poseArray;
		inputs.timestamps = timestampArray;
		inputs.visionStdDevs = visionStdArray;
		inputs.latency = latencyArray;
	}

	@Override
	public void updatePose(Pose2d pose) {
		visionSim.update(pose);
	}
}
