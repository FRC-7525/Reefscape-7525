package frc.robot.subsystems.VisionIOReal;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOReal implements VisionIO {
  private final CameraPoseEstimator[] poseEstimators;

  private Pose3d[] poseArray;
  private double[] timestampArray;
  private double[] visionStdArray;
  private double[] latencyArray;

  public VisionIOReal(CameraPoseEstimator[] poseEstimators) {
    this.poseEstimators = poseEstimators;

    poseArray = new Pose3d[poseEstimators.length];
    timestampArray = new double[poseEstimators.length];
    visionStdArray = new double[poseEstimators.length * 3];
    latencyArray = new double[poseEstimators.length];
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
}