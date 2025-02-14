package frc.robot.Subsystems.VisionIOSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import static frc.robot.Subsystems.Vision.VisionConstants.*;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstants.CameraResolution;
import frc.robot.Constants.AprilTagConstants.DoryCameras;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
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

  private Pose3d[] poseArray = new Pose3d[4];
  private double[] timestampArray = new double[4];
  private double[] visionStdArray = new double[4 * 3];
  private double[] latencyArray;

  public AprilTagVisionIOSim() {
    PhotonCamera frontLeft = new PhotonCamera(DoryCameras.frontLeftName);
    PhotonCamera frontRight = new PhotonCamera(DoryCameras.frontRightName);
    PhotonCamera backLeft = new PhotonCamera(DoryCameras.backLeftName);
    PhotonCamera backRight = new PhotonCamera(DoryCameras.backRightName);

    frontLeftPose =
        new CameraPoseEstimator(
            frontLeft,
            DoryCameras.frontLeftFromRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);
    frontRightPose =
        new CameraPoseEstimator(
            frontRight,
            DoryCameras.frontRightFromRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);
    backLeftPose =
        new CameraPoseEstimator(
            backLeft,
            DoryCameras.backLeftFromRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);
    backRightPose =
        new CameraPoseEstimator(
            backRight,
            DoryCameras.backRightFromRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);

    this.poseEstimators =
        new CameraPoseEstimator[] {frontLeftPose, frontRightPose, backLeftPose, backRightPose};

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(AprilTagConstants.aprilTagFieldLayout);

    SimCameraProperties cameraProps = new SimCameraProperties(); // Corresponds to high-res cameras
    cameraProps.setCalibration(1600, 1200, Rotation2d.fromDegrees(75));
    cameraProps.setCalibError(0.25, 0.10);
    cameraProps.setFPS(15);
    cameraProps.setAvgLatencyMs(50);
    cameraProps.setLatencyStdDevMs(15);

    frontLeftSim = new PhotonCameraSim(frontLeft, cameraProps);
    frontRightSim = new PhotonCameraSim(frontRight, cameraProps);
    backLeftSim = new PhotonCameraSim(backLeft, cameraProps);
    backRightSim = new PhotonCameraSim(backRight, cameraProps);

    visionSim.addCamera(frontLeftSim, DoryCameras.frontLeftFromRobot);
    visionSim.addCamera(frontRightSim, DoryCameras.frontRightFromRobot);
    visionSim.addCamera(backLeftSim, DoryCameras.backLeftFromRobot);
    visionSim.addCamera(backRightSim, DoryCameras.backRightFromRobot);

    frontLeftSim.enableDrawWireframe(true);
    frontRightSim.enableDrawWireframe(true);
    backLeftSim.enableDrawWireframe(true);
    backRightSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    getEstimatedPoseUpdates(
        poseEstimators, poseArray, timestampArray, visionStdArray, latencyArray);
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