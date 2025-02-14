package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Subsystems.Vision.VisionConstants.*;
import static frc.robot.subsystems.Vision.VisionConstants.BACK_CAMERA_NAME;
import static frc.robot.subsystems.Vision.VisionConstants.BACK_RESOLUTION;
import static frc.robot.subsystems.Vision.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.subsystems.Vision.VisionConstants.FRONT_RESOLUTION;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_BACK_CAMERA;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_FRONT_CAMERA;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import com.google.flatbuffers.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.Vision.VisionIO.CameraPoseEstimator;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team7525.misc.VisionUtil.CameraResolution;

public class Vision extends SubsystemBase {

	VisionIO io;
	SwerveDrivePoseEstimator drivePoseEstimator;
	private final VisionIOInputsAutoLogged VisionInputs = new VisionIOInputsAutoLogged();

	@AutoLogOutput
	public boolean useVision = true;

	/**
	 * Constructor for the AprilTagVision subsystem. This is intended to be
	 * instantiated in Drive.
	 *
	 * @param swerveDrivePoseEstimator A {@link SwerveDrivePoseEstimator} to write
	 *                                 vision data to.
	 *                                 Probably from Drive.
	 */
	public Vision(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
		CameraPoseEstimator[] visionPoseEstimators = {};
		switch (currentRobot) {
			case NAUTILUS:
				visionPoseEstimators = new CameraPoseEstimator[] {
					new CameraPoseEstimator(new PhotonCamera(BACK_CAMERA_NAME), ROBOT_TO_BACK_CAMERA, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BACK_RESOLUTION),
					new CameraPoseEstimator(new PhotonCamera(FRONT_CAMERA_NAME), ROBOT_TO_FRONT_CAMERA, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FRONT_RESOLUTION),
				};
			case DORY:
				visionPoseEstimators = new CameraPoseEstimator[] {
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.frontLeftName), DoryCameras.frontLeftFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.frontRightName), DoryCameras.frontRightFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.backLeftName), DoryCameras.backLeftFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.backRightName), DoryCameras.backRightFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
				};
		}

		switch (Constants.currentMode) {
			case REAL:
				this.io = new AprilTagVisionIOReal(visionPoseEstimators);
				break;
			case REPLAY:
				this.io = new AprilTagVisionIO() {};
				break;
			case SIM:
				this.io = new AprilTagVisionIOSim();
				break;
		}

		// For sim. If we do a full drivetrain sim move this there so it'll update as
		// the simulated
		// robot moves.
		io.updatePose(new Pose2d(1.06, 2.50, new Rotation2d(Radians.convertFrom(-114.0, Radians))));

		this.drivePoseEstimator = swerveDrivePoseEstimator;
	}

	@Override
	public void periodic() {
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

				Logger.recordOutput("Drive/AprilTagPose" + i, aprilTagVisionInputs.visionPoses[i].toPose2d());
				Logger.recordOutput("Drive/AprilTagStdDevs" + i, Arrays.copyOfRange(aprilTagVisionInputs.visionStdDevs, 3 * i, 3 * i + 3));
				Logger.recordOutput("Drive/AprilTagTimestamps" + i, aprilTagVisionInputs.timestamps[i]);

				if (useVision) {
					drivePoseEstimator.addVisionMeasurement(aprilTagVisionInputs.visionPoses[i].toPose2d(), aprilTagVisionInputs.timestamps[i], VecBuilder.fill(aprilTagVisionInputs.visionStdDevs[3 * i], aprilTagVisionInputs.visionStdDevs[3 * i + 1], aprilTagVisionInputs.visionStdDevs[3 * i + 2]));
				}
			} else {
				Logger.recordOutput("Drive/AprilTagPose" + i, new Pose2d());
				Logger.recordOutput("Drive/AprilTagStdDevs" + i, new double[] { 0.0, 0.0, 0.0 });
			}
		}
	}
}
