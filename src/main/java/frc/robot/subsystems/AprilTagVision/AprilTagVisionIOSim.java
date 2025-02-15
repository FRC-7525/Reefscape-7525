package frc.robot.Subsystems.AprilTagVision;

import static frc.robot.Subsystems.AprilTagVision.AprilTagConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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

	private Pose3d[] poseArray;
	private double[] timestampArray;
	private double[] visionStdArray;
	private double[] latencyArray;

	public AprilTagVisionIOSim() {
		PhotonCamera frontRight = new PhotonCamera(FRONT_CAM_NAME);
		PhotonCamera backLeft = new PhotonCamera(BACK_CAM_NAME);

		frontRightPose = new CameraPoseEstimator(frontRight, ROBOT_TO_FRONT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES);
		backLeftPose = new CameraPoseEstimator(backLeft, ROBOT_TO_BACK_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES);
		this.poseEstimators = new CameraPoseEstimator[] { frontRightPose, backLeftPose };

		poseArray = new Pose3d[poseEstimators.length];
		timestampArray = new double[poseEstimators.length];
		visionStdArray = new double[poseEstimators.length * 3];
		latencyArray = new double[poseEstimators.length];

		visionSim = new VisionSystemSim("main");
		visionSim.addAprilTags(AprilTagConstants.aprilTagFieldLayout);

		SimCameraProperties cameraProps = new SimCameraProperties(); // Corresponds to high-res cameras
		cameraProps.setCalibration(1600, 1200, Rotation2d.fromDegrees(86));
		cameraProps.setCalibError(0.25, 0.10);
		cameraProps.setFPS(15);
		cameraProps.setAvgLatencyMs(50);
		cameraProps.setLatencyStdDevMs(15);

		frontRightSim = new PhotonCameraSim(frontRight, cameraProps);
		backLeftSim = new PhotonCameraSim(backLeft, cameraProps);

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
