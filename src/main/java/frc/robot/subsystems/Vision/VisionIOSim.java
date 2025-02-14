package frc.robot.Subsystems.VisionIOSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import static frc.robot.Subsystems.Vision.VisionConstants.*;
import static frc.robot.subsystems.Vision.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.subsystems.Vision.VisionConstants.BACK_CAMERA_NAME;
import static frc.robot.subsystems.Vision.VisionConstants.BACK_RESOLUTION;
import static frc.robot.subsystems.Vision.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.subsystems.Vision.VisionConstants.FRONT_RESOLUTION;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_BACK_CAMERA;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_FRONT_CAMERA;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSim;

  // Camera simulators
  private PhotonCameraSim frontSim;
  private PhotonCameraSim backSim;


  // Pose estimators
  private CameraPoseEstimator frontPose;
  private CameraPoseEstimator backPose;


  private CameraPoseEstimator[] poseEstimators;

  private Pose3d[] poseArray = new Pose3d[4];
  private double[] timestampArray = new double[4];
  private double[] visionStdArray = new double[4 * 3];
  private double[] latencyArray;

  public VisionIOSim() {
    PhotonCamera frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    PhotonCamera backCamera = new PhotonCamera(BACK_CAMERA_NAME);


    frontPose =
        new CameraPoseEstimator(
            frontCamera,
            ROBOT_TO_FRONT_CAMERA,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            FRONT_RESOLUTION); //idk why resolution is not worked
    backPose =
        new CameraPoseEstimator(
            backCamera,
            ROBOT_TO_BACK_CAMERA,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            BACK_RESOLUTION);//idk why resolution is not worked
    

    this.poseEstimators =
        new CameraPoseEstimator[] {frontPose, backPose};

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);

    SimCameraProperties cameraProps = new SimCameraProperties(); // Corresponds to high-res cameras
    cameraProps.setCalibration(1600, 1200, Rotation2d.fromDegrees(75));
    cameraProps.setCalibError(0.25, 0.10);
    cameraProps.setFPS(15);
    cameraProps.setAvgLatencyMs(50);
    cameraProps.setLatencyStdDevMs(15);

    frontSim = new PhotonCameraSim(frontCamera, cameraProps);
    backSim = new PhotonCameraSim(backCamera, cameraProps);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
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