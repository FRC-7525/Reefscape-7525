package frc.robot.Subsystems.Vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.PoseObservation;
import frc.robot.Subsystems.Vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

	// Akit my savior
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;

	List<Pose3d> allTagPoses = new LinkedList<>();
	List<Pose3d> allRobotPoses = new LinkedList<>();
	List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
	List<Pose3d> allRobotPosesRejected = new LinkedList<>();

	private static Vision instance;

	public static Vision getInstance() {
		if (instance == null) {
			instance = new Vision(
				switch (ROBOT_MODE) {
					case REAL -> new VisionIO[] {
						new VisionIOPhotonVision(FRONT_LEFT_CAM_NAME, ROBOT_TO_FRONT_LEFT_CAMERA),
						new VisionIOPhotonVision(FRONT_RIGHT_CAM_NAME, ROBOT_TO_FRONT_RIGHT_CAMERA),
						// new VisionIOPhotonVision(BACK_LEFT_CAM_NAME, ROBOT_TO_BACK_LEFT_CAMERA),
						new VisionIOPhotonVision(BACK_RIGHT_CAM_NAME, ROBOT_TO_BACK_RIGHT_CAMERA),
					};
					case SIM -> new VisionIO[] {
						new VisionIOPhotonVisionSim(FRONT_LEFT_CAM_NAME, ROBOT_TO_FRONT_LEFT_CAMERA, Drive.getInstance()::getPose),
						new VisionIOPhotonVisionSim(FRONT_RIGHT_CAM_NAME, ROBOT_TO_FRONT_RIGHT_CAMERA, Drive.getInstance()::getPose),
						new VisionIOPhotonVisionSim(BACK_LEFT_CAM_NAME, ROBOT_TO_BACK_LEFT_CAMERA, Drive.getInstance()::getPose),
						new VisionIOPhotonVisionSim(BACK_RIGHT_CAM_NAME, ROBOT_TO_BACK_RIGHT_CAMERA, Drive.getInstance()::getPose),
					};
					case TESTING -> new VisionIO[] { 
						new VisionIOPhotonVision(FRONT_RIGHT_CAM_NAME, ROBOT_TO_FRONT_LEFT_CAMERA),
						new VisionIOPhotonVision(BACK_LEFT_CAM_NAME, ROBOT_TO_BACK_LEFT_CAMERA)
					};
				}
			);
		}
		return instance;
	}

	private Vision(VisionIO... io) {
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing
	 * with vision.
	 *
	 * @param cameraIndex The index of the camera to use.
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}
		
		// Initialize logging values
		allTagPoses = new LinkedList<>();
		allRobotPoses = new LinkedList<>();
		allRobotPosesAccepted = new LinkedList<>();
		allRobotPosesRejected = new LinkedList<>();

		processVision();
	}

	private void processVision() {
		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				boolean rejectPose = shouldBeRejected(observation);

				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Tag Count", observation.tagCount() == 0);
				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Ambiguous", (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity));
				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field X", observation.pose().getX() < 0.0 || observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength());
				Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field Y", observation.pose().getY() < 0.0 || observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth());

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) continue;

				// Calculate standard deviations
				double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = linearStdDevBaseline * stdDevFactor;
				double angularStdDev = angularStdDevBaseline * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= linearStdDevMegatag2Factor;
					angularStdDev *= angularStdDevMegatag2Factor;
				}
				if (cameraIndex < cameraStdDevFactors.length) {
					linearStdDev *= cameraStdDevFactors[cameraIndex];
					angularStdDev *= cameraStdDevFactors[cameraIndex];
				}



				// Send vision observation
				Drive.getInstance().addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			// Log camera datadata
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}
	}

	private boolean shouldBeRejected(PoseObservation observation) {
		Logger.recordOutput("Test", Math.abs(Units.radiansToDegrees(Drive.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond)));
		return 
			observation.tagCount() == 0 || // Must have at least one tag
			(observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) || // Cannot be high ambiguity
			Math.abs(observation.pose().getZ()) > maxZError || // Must have realistic Z coordinate
			// Must be within the field boundaries
			observation.pose().getX() < 0.0 ||
			observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength() ||
			observation.pose().getY() < 0.0 ||
			observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth() ||
			((observation.pose().getRotation().toRotation2d().getDegrees() - Drive.getInstance().getPose().getRotation().getDegrees() + 180) % 360) - 180 > GYRO_REPROJECTION_MARGIN.in(Degree) || // Must be somewhat accurate to gyro
			Math.abs(Units.radiansToDegrees(Drive.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond)) > MAX_ANGULAR_VELOCITY.in(DegreesPerSecond) ; // Robot must not be rotating rapidly
	}
}
