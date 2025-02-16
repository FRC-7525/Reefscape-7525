package frc.robot.Subsystems.AprilTagVision;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.AprilTagVision.AprilTagConstants.*;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.AprilTagVision.AprilTagConstants.CameraResolution;
import frc.robot.Subsystems.AprilTagVision.AprilTagVisionIO.CameraPoseEstimator;
import frc.robot.Subsystems.Drive.Drive;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class AprilTagVision extends SubsystemBase {

	// AEMBOT my savior
	AprilTagVisionIO io;
	private final AprilTagVisionIOInputsAutoLogged aprilTagVisionInputs = new AprilTagVisionIOInputsAutoLogged();

	@AutoLogOutput
	public boolean useVision = true;

	private static AprilTagVision instance;

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
	public AprilTagVision() {
		CameraPoseEstimator[] visionPoseEstimators = new CameraPoseEstimator[] {
			new CameraPoseEstimator(new PhotonCamera(FRONT_LEFT_CAM_NAME), ROBOT_TO_FRONT_LEFT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES),
			new CameraPoseEstimator(new PhotonCamera(FRONT_RIGHT_CAM_NAME), ROBOT_TO_FRONT_RIGHT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES),
			new CameraPoseEstimator(new PhotonCamera(BACK_LEFT_CAM_NAME), ROBOT_TO_BACK_LEFT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES),
			new CameraPoseEstimator(new PhotonCamera(BACK_RIGHT_CAM_NAME), ROBOT_TO_BACK_RIGHT_CAMERA, AprilTagConstants.poseStrategy, CameraResolution.HIGH_RES),
		};

		switch (ROBOT_MODE) {
			case REAL:
				this.io = new AprilTagVisionIOReal(visionPoseEstimators);
				break;
			case TESTING:
				this.io = new AprilTagVisionIOReal(visionPoseEstimators);
				break;
			case SIM:
				this.io = new AprilTagVisionIOSim();
				break;
		}
	}

	@Override
	public void periodic() {
		io.updatePose(Drive.getInstance().getPose());
		io.updateInputs(aprilTagVisionInputs);
		Logger.processInputs("AprilTagVision", aprilTagVisionInputs);

		for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
			if ( // Bounds check the estimated robot pose is actually on the field
				aprilTagVisionInputs.timestamps[i] >= 1.0 &&
				Math.abs(aprilTagVisionInputs.visionPoses[i].getZ()) < 1.0 &&
				aprilTagVisionInputs.visionPoses[i].getX() > 0 &&
				aprilTagVisionInputs.visionPoses[i].getX() < aprilTagFieldLayout.getFieldLength() &&
				aprilTagVisionInputs.visionPoses[i].getY() > 0 &&
				aprilTagVisionInputs.visionPoses[i].getY() < aprilTagFieldLayout.getFieldWidth() &&
				aprilTagVisionInputs.visionPoses[i].getRotation().getX() < 0.2 &&
				aprilTagVisionInputs.visionPoses[i].getRotation().getY() < 0.2
			) {
				if (aprilTagVisionInputs.timestamps[i] > (getTimestamp() / 1.0e6)) {
					aprilTagVisionInputs.timestamps[i] = (getTimestamp() / 1.0e6) - aprilTagVisionInputs.latency[i];
				}

				Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTagPose" + i, aprilTagVisionInputs.visionPoses[i].toPose2d());
				Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTagStdDevs" + i, Arrays.copyOfRange(aprilTagVisionInputs.visionStdDevs, 3 * i, 3 * i + 3));
				Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTagTimestamps" + i, aprilTagVisionInputs.timestamps[i]);

				if (useVision) {
					Drive.getInstance().addVisionMeasurement(aprilTagVisionInputs.visionPoses[i].toPose2d(), aprilTagVisionInputs.timestamps[i], VecBuilder.fill(aprilTagVisionInputs.visionStdDevs[3 * i], aprilTagVisionInputs.visionStdDevs[3 * i + 1], aprilTagVisionInputs.visionStdDevs[3 * i + 2]));
				}
			} else {
				Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTagPose" + i, new Pose2d());
				Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTagStdDevs" + i, new double[] { 0.0, 0.0, 0.0 });
			}
		}
	}
}
