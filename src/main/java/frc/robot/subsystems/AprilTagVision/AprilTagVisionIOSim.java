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
	private PhotonCameraSim frontLeftSim;
	private PhotonCameraSim frontRightSim;
	private PhotonCameraSim backLeftSim;
	private PhotonCameraSim backRightSim;

	// Pose estimators
	private CameraPoseEstimator frontLeftPose;
	private CameraPoseEstimator frontRightPose;
	private CameraPoseEstimator backLeftPose;
	private CameraPoseEstimator backRightPose;

	private CameraPoseEstimator[] poseEstimators;

	private Pose3d[] poseArray;
	private double[] timestampArray;
	private double[] visionStdArray;
	private double[] latencyArray;

	public AprilTagVisionIOSim() {
		PhotonCamera frontLeft = new PhotonCamera(FRONT_LEFT_CAM_NAME);
		PhotonCamera frontRight = new PhotonCamera(FRONT_RIGHT_CAM_NAME);
		PhotonCamera backLeft = new PhotonCamera(BACK_LEFT_CAM_NAME);
		PhotonCamera backRight = new PhotonCamera(BACK_RIGHT_CAM_NAME);

		frontLeftPose = new CameraPoseEstimator(frontLeft, ROBOT_TO_FRONT_LEFT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES);
		frontRightPose = new CameraPoseEstimator(frontRight, ROBOT_TO_FRONT_RIGHT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES);
		backLeftPose = new CameraPoseEstimator(backLeft, ROBOT_TO_BACK_LEFT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES);
		backRightPose = new CameraPoseEstimator(backRight, ROBOT_TO_BACK_RIGHT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES);
		this.poseEstimators = new CameraPoseEstimator[] { frontLeftPose, frontRightPose, backLeftPose, backRightPose };

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

		frontLeftSim = new PhotonCameraSim(frontLeft, cameraProps);
		frontRightSim = new PhotonCameraSim(frontRight, cameraProps);
		backLeftSim = new PhotonCameraSim(backLeft, cameraProps);
		backRightSim = new PhotonCameraSim(backRight, cameraProps);

		visionSim.addCamera(frontLeftSim, ROBOT_TO_FRONT_LEFT_CAMERA);
		visionSim.addCamera(frontRightSim, ROBOT_TO_FRONT_RIGHT_CAMERA);
		visionSim.addCamera(backLeftSim, ROBOT_TO_BACK_LEFT_CAMERA);
		visionSim.addCamera(backRightSim, ROBOT_TO_BACK_RIGHT_CAMERA);

		frontLeftSim.enableDrawWireframe(true);
		frontRightSim.enableDrawWireframe(true);
		backLeftSim.enableDrawWireframe(true);
		backRightSim.enableDrawWireframe(true);
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
