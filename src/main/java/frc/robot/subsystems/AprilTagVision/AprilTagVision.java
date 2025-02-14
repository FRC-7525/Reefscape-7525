package frc.robot.Subsystems.AprilTagVision;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.AprilTagVision.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Subsystems.AprilTagVision.VisionConstants.BACK_CAMERA_NAME;
import static frc.robot.Subsystems.AprilTagVision.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.Subsystems.AprilTagVision.VisionConstants.POSE_STRATEGY;
import static frc.robot.Subsystems.AprilTagVision.VisionConstants.ROBOT_TO_BACK_CAMERA;
import static frc.robot.Subsystems.AprilTagVision.VisionConstants.ROBOT_TO_FRONT_CAMERA;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.AprilTagVision.AprilTagVisionIO.CameraPoseEstimator;
import frc.robot.Subsystems.AprilTagVision.VisionConstants.CameraResolution;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.AprilTagVisionIOInputsAutoLogged;

import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class AprilTagVision extends SubsystemBase {
  AprilTagVisionIO io;
  private final AprilTagVisionIOInputsAutoLogged aprilTagVisionInputs =
      new AprilTagVisionIOInputsAutoLogged();

  private static AprilTagVision instance;

  @AutoLogOutput public boolean useVision = true;

  public static AprilTagVision getInstance() {
    if (instance == null) {
      instance = new AprilTagVision();
    }
    return instance;
  }

  /**
   * Constructor for the AprilTagVision subsystem. This is intended to be instantiated in Drive.
   *
   * @param swerveDrivePoseEstimator A {@link SwerveDrivePoseEstimator} to write vision data to.
   *     Probably from Drive.
   */
  private AprilTagVision() {
    CameraPoseEstimator[] visionPoseEstimators = {};
    
      visionPoseEstimators =
          new CameraPoseEstimator[] {
            new CameraPoseEstimator(
                new PhotonCamera(FRONT_CAMERA_NAME),
                ROBOT_TO_FRONT_CAMERA,
                POSE_STRATEGY,
                CameraResolution.HIGH_RES),
            new CameraPoseEstimator(
                new PhotonCamera(BACK_CAMERA_NAME),
                ROBOT_TO_BACK_CAMERA,
                POSE_STRATEGY,
                CameraResolution.HIGH_RES),
          };
    

    switch (ROBOT_MODE) {
      case REAL:
        this.io = new AprilTagVisionIOReal(visionPoseEstimators);
        break;
      case TESTING:
        this.io = new AprilTagVisionIOReal(visionPoseEstimators) {};
        break;
      case SIM:
        this.io = new AprilTagVisionIOSim();
        break;
    }

    // For sim. If we do a full drivetrain sim move this there so it'll update as the simulated
    // robot moves.
    io.updatePose(new Pose2d(1.06, 2.50, new Rotation2d(Radians.convertFrom(-114.0, Radians))));
  }

  @Override
  public void periodic() {
    io.updateInputs(aprilTagVisionInputs);
    Logger.processInputs("AprilTagVision", aprilTagVisionInputs);

    for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
      if ( // Bounds check the estimated robot pose is actually on the field
      aprilTagVisionInputs.timestamps[i] >= 1.0
          && Math.abs(aprilTagVisionInputs.visionPoses[i].getZ()) < 1.0
          && aprilTagVisionInputs.visionPoses[i].getX() > 0
          && aprilTagVisionInputs.visionPoses[i].getX() < APRIL_TAG_FIELD_LAYOUT.getFieldLength()
          && aprilTagVisionInputs.visionPoses[i].getY() > 0
          && aprilTagVisionInputs.visionPoses[i].getY() < APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
          && aprilTagVisionInputs.visionPoses[i].getRotation().getX() < 0.2
          && aprilTagVisionInputs.visionPoses[i].getRotation().getY() < 0.2) {
        if (aprilTagVisionInputs.timestamps[i] > (getTimestamp() / 1.0e6)) {
          aprilTagVisionInputs.timestamps[i] =
              (getTimestamp() / 1.0e6) - aprilTagVisionInputs.latency[i];
        }

        Logger.recordOutput(
            "Vision/AprilTagPose" + i, aprilTagVisionInputs.visionPoses[i].toPose2d());
        Logger.recordOutput(
            "Vision/AprilTagStdDevs" + i,
            Arrays.copyOfRange(aprilTagVisionInputs.visionStdDevs, 3 * i, 3 * i + 3));
        Logger.recordOutput("Vision/AprilTagTimestamps" + i, aprilTagVisionInputs.timestamps[i]);

        if (useVision) {
          Drive.getInstance().addVisionMeasurement(
              aprilTagVisionInputs.visionPoses[i].toPose2d(),
              aprilTagVisionInputs.timestamps[i],
              VecBuilder.fill(
                  aprilTagVisionInputs.visionStdDevs[3 * i],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 1],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 2]));
        }
      } else {
        Logger.recordOutput("Vision/AprilTagPose" + i, new Pose2d());
        Logger.recordOutput("Vision/AprilTagStdDevs" + i, new double[] {0.0, 0.0, 0.0});
      }
    }
  }
}
