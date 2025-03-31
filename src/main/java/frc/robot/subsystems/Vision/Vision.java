package frc.robot.Subsystems.Vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.SubsystemManager.SubsystemManager;
import frc.robot.SubsystemManager.SubsystemManagerStates;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionConstants.VisionMeasurment;
import frc.robot.Subsystems.Vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;


public class Vision {

	// Akit my savior
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;
	private final Consumer<VisionMeasurment> estimatorConsumer;
	private String subsystemName;

	List<Pose3d> allTagPoses = new LinkedList<>();
	List<Pose3d> allRobotPoses = new LinkedList<>();
	List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
	List<Pose3d> allRobotPosesRejected = new LinkedList<>();

	Set<Short> allianceReefTag = Robot.isRedAlliance ? RED_REEF_TAGS : BLUE_REEF_TAGS;

	public Vision(String subsystemName, Consumer<VisionMeasurment> estimateConsumer, VisionIO... io) {
		this.io = io;
		this.subsystemName = subsystemName;
		this.estimatorConsumer = estimateConsumer;

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

	public void periodic() {
		allianceReefTag = Robot.isRedAlliance ? RED_REEF_TAGS : BLUE_REEF_TAGS;

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
		// Log summary data
		Logger.recordOutput(subsystemName+ "/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput(subsystemName+ "/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput(subsystemName + "/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput(subsystemName + "/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
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

				Logger.recordOutput(subsystemName +"/Camera" + Integer.toString(cameraIndex) + "/Tag Count", observation.tagCount() == 0);
				Logger.recordOutput(subsystemName +"/Camera" + Integer.toString(cameraIndex) + "/Ambiguous", (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity));
				Logger.recordOutput(subsystemName +"/Camera" + Integer.toString(cameraIndex) + "/Outside of Field X", observation.pose().getX() < 0.0 || observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength());
				Logger.recordOutput(subsystemName+ "/Camera" + Integer.toString(cameraIndex) + "/Outside of Field Y", observation.pose().getY() < 0.0 || observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth());

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
				// double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				// double linearStdDev = linearStdDevBaseline * stdDevFactor;
				// double angularStdDev = angularStdDevBaseline * stdDevFactor;
				// if (observation.type() == PoseObservationType.MEGATAG_2) {
				// 	linearStdDev *= linearStdDevMegatag2Factor;
				// 	angularStdDev *= angularStdDevMegatag2Factor;
				// }
				// if (cameraIndex < cameraStdDevFactors.length) {
				// 	linearStdDev *= cameraStdDevFactors[cameraIndex];
				// 	angularStdDev *= cameraStdDevFactors[cameraIndex];
				// }

				//254 standard dev
				Matrix<N3, N1> visionStandardDev = calculateStandardDev(observation);


				// Send vision observation
				estimatorConsumer.accept(new VisionMeasurment(observation.pose().toPose2d(), observation.timestamp(), visionStandardDev));

				// Drive.getInstance().addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp(), visionStandardDev);
			}

			// Log camera datadata
			Logger.recordOutput(subsystemName + "/Camera" + Integer.toString(cameraIndex) + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput(subsystemName + "/Camera" + Integer.toString(cameraIndex) + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput(subsystemName +"/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput(subsystemName +"/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}
	}

	private boolean shouldBeRejected(PoseObservation observation) {
		boolean observedBarge = false;
		boolean observedSource = false;

		for (Short tagObserved : observation.tagsObserved()) {
			if (APRIL_TAG_IGNORE.contains(tagObserved)) {
				observedBarge = true;
				break;
			}
			if (observedBarge) break;
		}

		for (Short tagObserved : observation.tagsObserved()) {
			if (SOURCE_TAGS.contains(tagObserved)) {
				observedSource = true;
				break;
			}
			if (observedSource) break;
		}

		return (
			observation.tagCount() == 0 || // Must have at least one tag
			(observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) || // Cannot be high ambiguity
			Math.abs(observation.pose().getZ()) > maxZError || // Must have realistic Z coordinate
			// Must be within the field boundaries
			observation.pose().getX() <
			0.0 ||
			observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength() ||
			observation.pose().getY() < 0.0 ||
			observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth() ||
			observedBarge || // Barge poses mess up vision
			(observedSource && SubsystemManager.getInstance().getState() == SubsystemManagerStates.AUTO_ALIGN_CLOSE) || // Source can mess up AA
			Math.abs(Units.radiansToDegrees(Drive.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond)) > MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)
		); // Robot must not be rotating rapidly
	}

	public Matrix<N3, N1> calculateStandardDev(PoseObservation observation) {
		double xyStds;
		double degStds;
		if (observation.tagCount() == 1) {
			double poseDifference = observation.pose().getTranslation().toTranslation2d().getDistance(Drive.getInstance().getPose().getTranslation());
			if (seenReefTags(observation) && observation.avgTagArea() > 0.2) {
				xyStds = 0.5;
			}
			// 1 target with large area and close to estimated pose
			else if (observation.avgTagArea() > 0.8 && poseDifference < 0.5) {
				xyStds = 0.5;
			}
			// 1 target farther away and estimated pose is close
			else if (observation.avgTagArea() > 0.1 && poseDifference < 0.3) {
				xyStds = 1.0;
			} else {
				xyStds = 2.0;
			}
			return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(50)); // I dont even know, ts so random
		} else {
			xyStds = 0.5;
			degStds = 6;
			return VecBuilder.fill(xyStds, xyStds, degStds);
		}
	}

	private boolean seenReefTags(PoseObservation observation) {
		return allianceReefTag.contains(observation.tagsObserved().toArray()[0]);
	}
}
