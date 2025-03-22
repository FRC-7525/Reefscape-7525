package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.AutoAlign.AutoAlignConstants.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Robot;
import frc.robot.SubsystemManager.SubsystemManager;
import frc.robot.Subsystems.Drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team7525.autoAlign.RepulsorFieldPlanner;
import org.team7525.subsystem.Subsystem;

public class AutoAlign extends Subsystem<AutoAlignStates> {

	// Note for reviewer - ima put everything in constants once I finalize AA, this is a rough draft expect 200 more LOC

	private static AutoAlign instance;

	private final Drive drive = Drive.getInstance();

	private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner(new ArrayList<>(), new ArrayList<>(), (ROBOT_MODE == RobotMode.SIM));

	private final ProfiledPIDController rotationController;
	private final ProfiledPIDController translationalController;

	private PIDController repulsorTranslationController;
	private PIDController repulsorRotationalController;

	private Debouncer autoAlignDebouncer;

	private double driveErrorAbs;
	private double thetaErrorAbs;
	private double ffMinRadius = 0.2, ffMaxRadius = 1.0;

	private Pose2d reefPose = REEF_POSE;
	private List<Translation2d> reefVertices = REEF_VERTICES;
	private List<Translation2d> reefEdges = REEF_EDGES;

	private Pose2d targetPose;
	private Pose2d goalPose;
	private Pose2d interpolatedPose;
	private double interpolatedDistanceFromReef;
	private boolean repulsorActivated;
	private double timer = -1;

	private AutoAlign() {
		super("AutoAlign", AutoAlignStates.OFF);
		// FF impl (if 254 has magic number so can I hehehe)
		this.rotationController = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2), 0.02);

		this.translationalController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(9), 7), 0.02);

		this.repulsorTranslationController = REPULSOR_TRANSLATIONAL_CONTROLLER.get();
		this.repulsorRotationalController = REPULSOR_ROTATIONAL_CONTROLLER.get();

		this.repulsorTranslationController.setTolerance(TRANSLATIONAL_COMPONENT_ERROR_MARGIN.in(Meters));
		this.repulsorRotationalController.setTolerance(ANGLE_ERROR_MARGIN.in(Radians));
		this.rotationController.setTolerance(ANGLE_ERROR_MARGIN.in(Radians));

		SmartDashboard.putData("FF Rot Controller", rotationController);
		SmartDashboard.putData("FF TRans", translationalController);
		SmartDashboard.putData("Repulsor Trans", repulsorTranslationController);
		SmartDashboard.putData("Repulsor Rot", repulsorRotationalController);

		// Configure continuous input for rotation controllers
		this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
		this.repulsorRotationalController.enableContinuousInput(MIN_HEADING_ANGLE.in(Radians), MAX_HEADING_ANGLE.in(Radians));

		this.autoAlignDebouncer = new Debouncer(0.5, DebounceType.kRising);
		this.repulsorActivated = false;
		this.targetPose = new Pose2d();
		this.goalPose = new Pose2d();
	}

	public static AutoAlign getInstance() {
		if (instance == null) {
			instance = new AutoAlign();
		}
		return instance;
	}

	@Override
	protected void runState() {
		// Get the target pose from the current state
		goalPose = getState().getTargetPose();
		targetPose = goalPose;

		// Update current pose information
		Pose2d currentPose = drive.getPose();
		logOutput();

		// Angle at reef iirc
		if (!readyForClose()) {
			Translation2d temp = reefPose.getTranslation().minus(currentPose.getTranslation());
			targetPose = new Pose2d(targetPose.getTranslation(), Rotation2d.fromRadians(Math.atan2(temp.getY(), temp.getX())));
		}
	}

	// ACTUALLY DRIVING

	// 254's implementation of the braindead auto-align
	public void executeScaledFeedforwardAutoAlign() {
		Logger.recordOutput("AutoAlign/ProfiledVelSetpoint", translationalController.getSetpoint().velocity);
		Logger.recordOutput("AutoAlign/ProfiledPosSetpoint", translationalController.getSetpoint().position);
		Pose2d currentPose = drive.getPose();
		// Apply scalar drive with feedforward
		double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
		double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

		driveErrorAbs = currentDistance;
		translationalController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()), translationalController.getSetpoint().velocity);

		// Calculate translation velocity scalar with PID and FF scaling
		double translationVelocityScalar = translationalController.getSetpoint().velocity * ffScaler + translationalController.calculate(driveErrorAbs, 0.0);

		if (currentDistance < translationalController.getPositionTolerance()) {
			translationVelocityScalar = 0;
		}

		// Calculate rotation velocity with PID and FF scaling
		double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler + rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

		thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
		if (thetaErrorAbs < rotationController.getPositionTolerance()) {
			thetaVelocity = 0;
		}

		// Calculate final translation velocity
		var translationVelocity = MathHelpers.pose2dFromRotation(currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()).transformBy(MathHelpers.transform2dFromTranslation(new Translation2d(translationVelocityScalar, 0.0))).getTranslation();

		// Apply drive commands with alliance compensation
		if (Robot.isRedAlliance) {
			drive.driveFieldRelative(-translationVelocity.getX(), -translationVelocity.getY(), thetaVelocity, false, false);
		} else {
			drive.driveFieldRelative(translationVelocity.getX(), translationVelocity.getY(), thetaVelocity, false, false);
		}
	}

	public void executeRepulsorAutoAlign() {
		Pose2d currentPose = drive.getPose();
		// Set repulsor goal and get command
		repulsor.setGoal(targetPose.getTranslation());
		SwerveSample sample = repulsor.getCmd(currentPose, drive.getRobotRelativeSpeeds(), MAX_SPEED.in(MetersPerSecond), USE_GOAL, targetPose.getRotation());

		// Extract and modify chassis speeds with additional control
		var targetSpeeds = sample.getChassisSpeeds();
		targetSpeeds.vxMetersPerSecond += repulsorTranslationController.calculate(currentPose.getX(), sample.x);
		targetSpeeds.vyMetersPerSecond += repulsorTranslationController.calculate(currentPose.getY(), sample.y);
		targetSpeeds.omegaRadiansPerSecond += repulsorRotationalController.calculate(currentPose.getRotation().getRadians(), sample.heading);

		// No more race conditions :Sob:
		if (Robot.isRedAlliance) {
			drive.driveFieldRelative(-targetSpeeds.vxMetersPerSecond, -targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false, false);
		} else {
			drive.driveFieldRelative(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false, false);
		}
	}

	// UTIL STUFF

	// LOS
	// TODO: Profile this to see how resource intensive it is
	public boolean willCollideWithReef() {
		Pose2d currentPose = drive.getPose();

		double t = calculateClosestPoint(currentPose, targetPose);
		interpolatedPose = new Pose2d(currentPose.getTranslation().plus(targetPose.getTranslation().minus(currentPose.getTranslation()).times(t)), new Rotation2d());

		//Too many instantiations and calculations?
		List<Translation2d> interpolatedVertices = new ArrayList<Translation2d>();
		for (int i = 0; i < ROBOT_VERTICES.size(); i++) {
			interpolatedVertices.add(ROBOT_VERTICES.get(i).plus(interpolatedPose.getTranslation()).rotateAround(interpolatedPose.getTranslation(), interpolatedPose.getRotation()));
		}

		List<Translation2d> robotEdges = List.of(
			interpolatedVertices.get(1).minus(interpolatedVertices.get(0)),
			interpolatedVertices.get(2).minus(interpolatedVertices.get(1)),
			interpolatedVertices.get(3).minus(interpolatedVertices.get(2)),
			interpolatedVertices.get(0).minus(interpolatedVertices.get(3))
		);

		//TODO: Probably better way to implement SAT here
		Translation2d perpendicularLine = null;
		List<Translation2d> perpendicularStack = new ArrayList<Translation2d>();
		double dot = 0;
		double amin = Double.NaN;
		double amax = Double.NaN;
		double bmin = Double.NaN;
		double bmax = Double.NaN;

		for (int i = 0; i < robotEdges.size(); i++) {
			perpendicularLine = new Translation2d(-robotEdges.get(i).getY(), robotEdges.get(i).getX());
			perpendicularStack.add(perpendicularLine);
		}

		for (int i = 0; i < reefEdges.size(); i++) {
			perpendicularLine = new Translation2d(-reefEdges.get(i).getY(), reefEdges.get(i).getX());
			perpendicularStack.add(perpendicularLine);
		}

		for (int i = 0; i < perpendicularStack.size(); i++) {
			amin = Double.NaN;
			amax = Double.NaN;
			bmin = Double.NaN;
			bmax = Double.NaN;

			for (int j = 0; j < interpolatedVertices.size(); j++) {
				dot = interpolatedVertices.get(j).getX() * perpendicularStack.get(i).getX() + interpolatedVertices.get(j).getY() * perpendicularStack.get(i).getY();

				if (Double.isNaN(amax) || dot > amax) {
					amax = dot;
				}

				if (Double.isNaN(amin) || dot < amin) {
					amin = dot;
				}
			}

			for (int j = 0; j < reefVertices.size(); j++) {
				dot = reefVertices.get(j).getX() * perpendicularStack.get(i).getX() + reefVertices.get(j).getY() * perpendicularStack.get(i).getY();

				if (Double.isNaN(bmax) || dot > bmax) {
					bmax = dot;
				}

				if (Double.isNaN(bmin) || dot < bmin) {
					bmin = dot;
				}
			}

			if ((amin < bmax && amin > bmin) || (bmin < amax && bmin > amin)) {
				continue;
			} else {
				Logger.recordOutput("AutoAlign/Gona hit reef", false);
				return false;
			}
		}

		Logger.recordOutput("AutoAlign/Gona hit reef", true);
		return true;
	}

	public boolean closeEnoughToIgnore() {
		Pose2d currentPose = drive.getPose();
		boolean withinDistance = Math.abs(targetPose.getTranslation().getDistance(currentPose.getTranslation())) < 0.3;
		Logger.recordOutput("AutoAlign/Within Distnace", withinDistance);
		return withinDistance;
	}

	// Used for LOS
	private double calculateClosestPoint(Pose2d startPose, Pose2d endPose) {
		double numeratorX = (reefPose.getX() - startPose.getX()) * (endPose.getX() - startPose.getX());
		double numeratorY = (reefPose.getY() - startPose.getY()) * (endPose.getY() - startPose.getY());
		double denominator = Math.pow(endPose.getX() - startPose.getX(), 2) + Math.pow(endPose.getY() - startPose.getY(), 2);

		return MathUtil.clamp((numeratorX + numeratorY) / denominator, 0, 1);
	}

	// near goal for source cause u need less tolerance
	public boolean nearGoalSource() {
		return (
			drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < DISTANCE_ERROR_MARGIN.in(Meters) * 2 &&
			(Math.abs(repulsorRotationalController.getError()) < (ANGLE_ERROR_MARGIN.in(Radians) * 2) || Math.abs(rotationController.getPositionError()) < (ANGLE_ERROR_MARGIN.in(Radians) * 2))
		);
	}

	// Near setpoint for final transition
	public boolean nearGoal() {
		return drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < DISTANCE_ERROR_MARGIN.in(Meters) && (Math.abs(repulsorRotationalController.getError()) < ANGLE_ERROR_MARGIN.in(Radians) || Math.abs(rotationController.getPositionError()) < ANGLE_ERROR_MARGIN.in(Radians));
	}

	// For close transitions (i.e. ready for elevator to go up)
	public boolean readyForClose() {
		return (drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < getState().getDistanceForCloseAA().in(Meters));
	}

	// IDK is this works lmfao
	public boolean nearGoalDebounced() {
		return autoAlignDebouncer.calculate(
			drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < DISTANCE_ERROR_MARGIN.in(Meters) && (Math.abs(repulsorRotationalController.getError()) < ANGLE_ERROR_MARGIN.in(Radians) || Math.abs(rotationController.getPositionError()) < ANGLE_ERROR_MARGIN.in(Radians))
		);
	}

	// We never really tested ts
	public boolean timedOut() {
		if (getStateTime() > 0.01 && drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) > TIMEOUT_DISTANCE_THRESHOLD) {
			timer = getStateTime();
		} else if (timer == -1 && getStateTime() > 0.01) { //Sometimes autoalign close starts within the threshold and so timer never gets set
			timer = getStateTime();
		}

		if (timer != -1 && getStateTime() - timer > TIMEOUT_THRESHOLD) {
			timer = -1;
			return true;
		} else {
			return false;
		}
	}

	// TODO: Call this on entrance/exit & impl 254s thingy, once entrance/exit are merged I'll actually do this
	public void resetPID() {
		Pose2d currentPose = drive.getPose();
		ChassisSpeeds currentSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), currentPose.getRotation());
		translationalController.reset(currentPose.getTranslation().getDistance(currentPose.getTranslation()), Math.min(0.0, -new Translation2d(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond).rotateBy(currentPose.getTranslation().getAngle().unaryMinus()).getX()));
	}

	private void logOutput() {
		if (!Robot.isReal()) {
			Pose2d[] arrowsArray = new Pose2d[] {};
			Logger.recordOutput("AutoAlign/Arrows", arrowsArray);
		}

		Logger.recordOutput("AutoAlign/UsingRepulsor", repulsorActivated);
		Logger.recordOutput("AutoAlign/InterpolatedDistanceFromReef", interpolatedDistanceFromReef);
		Logger.recordOutput("AutoAlign/InterpolatedPose", interpolatedPose);
		Logger.recordOutput("AutoAlign/TargetPose", targetPose);
		Logger.recordOutput("AutoAlign/GoalPose", goalPose);
		Logger.recordOutput("AutoAlign/DriveErrorAbs", driveErrorAbs);
		Logger.recordOutput("AutoAlign/ThetaErrorAbs", thetaErrorAbs);
	}

	public void setConstants() {
		reefPose = Robot.isRedAlliance ? new Pose2d(13.08, 4, new Rotation2d()) : new Pose2d(4.49, 4, new Rotation2d());
		reefVertices = List.of(
			new Translation2d(REEF_HITBOX.in(Meters) * Math.cos(Math.PI / 6), REEF_HITBOX.in(Meters) * Math.sin(Math.PI / 6)).plus(reefPose.getTranslation()),
			new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI) / 2), REEF_HITBOX.in(Meters) * Math.sin((Math.PI) / 2)).plus(reefPose.getTranslation()),
			new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI * 5) / 6), REEF_HITBOX.in(Meters) * Math.sin((Math.PI * 5) / 6)).plus(reefPose.getTranslation()),
			new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI * 7) / 6), REEF_HITBOX.in(Meters) * Math.sin((Math.PI * 7) / 6)).plus(reefPose.getTranslation()),
			new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI * 3) / 2), REEF_HITBOX.in(Meters) * Math.sin((Math.PI * 3) / 2)).plus(reefPose.getTranslation()),
			new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI * 11) / 6), REEF_HITBOX.in(Meters) * Math.sin((Math.PI * 11) / 6)).plus(reefPose.getTranslation())
		);
		reefEdges = List.of(
			reefVertices.get(1).minus(reefVertices.get(0)),
			reefVertices.get(2).minus(reefVertices.get(1)),
			reefVertices.get(3).minus(reefVertices.get(2)),
			reefVertices.get(4).minus(reefVertices.get(3)),
			reefVertices.get(5).minus(reefVertices.get(4)),
			reefVertices.get(0).minus(reefVertices.get(5))
		);
	}

	@Override
	protected void stateExit() {
		resetPID();
	}
}
