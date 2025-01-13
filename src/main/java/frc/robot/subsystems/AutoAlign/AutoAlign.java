package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Subsystems.AutoAlign.AutoAlignConstants.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.Drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.team7525.autoAlign.RepulsorFieldPlanner;
import org.team7525.subsystem.Subsystem;

public class AutoAlign extends Subsystem<AutoAlignStates> {

	private static AutoAlign instance;

	private final Drive drive = Drive.getInstance();
	private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();

	private PIDController translationController;
	private PIDController rotationController;
	private PIDController repulsionTranslationController;
	private PIDController repulsionRotationController;

	private Pose2d targetPose;
	private Pose2d reefPose = REEF_POSE;
	private Pose2d interpolatedPose;
	private double interpolatedDistanceFromReef;
	private boolean repulsorActivated;

	private AutoAlign() {
		super("AutoAlign", AutoAlignStates.OFF);
		// PID Config
		this.translationController = TRANSLATIONAL_CONTROLLER.get();
		this.rotationController = ROTATIONAL_CONTROLLER.get();
		this.repulsionTranslationController = REPULSOR_TRANSLATIONAL_CONTROLLER.get();
		this.repulsionRotationController = REPULSOR_ROTATIONAL_CONTROLLER.get();
	}

	public static AutoAlign getInstance() {
		if (instance == null) {
			instance = new AutoAlign();
		}
		return instance;
	}

	@Override
	protected void runState() {
		logOutput();

		if (getState() == AutoAlignStates.OFF) return;

		targetPose = getState().getTargetPose();

		// if there is no collision, it will go to braindead AA, or use repulsor to
		// avoid the collision
		if (!checkForReefCollision()) {
			braindeadAutoAlign();
			repulsorActivated = false;
		} else {
			repulsor.setGoal(targetPose.getTranslation());
			repulsorAutoAlign(drive.getPose(), repulsor.getCmd(drive.getPose(), drive.getRobotRelativeSpeeds(), MAX_SPEED.in(MetersPerSecond), USE_GOAL, targetPose.getRotation()));
			repulsorActivated = true;
		}
	}

	private void braindeadAutoAlign() {
		rotationController.enableContinuousInput(MIN_HEADING_ANGLE.in(Degrees), MAX_HEADING_ANGLE.in(Degrees));
		Pose2d drivePose = drive.getPose();

		// idk why applied needs to be negative but it works if it is negative 💀
		// update: it's negative because otto is a bum and can't set up his sim properly
		double xApplied = -repulsionTranslationController.calculate(drivePose.getX(), targetPose.getX());
		double yApplied = -repulsionTranslationController.calculate(drivePose.getY(), targetPose.getY());
		double rotationApplied = repulsionRotationController.calculate(drivePose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
		drive.driveFieldRelative(xApplied, yApplied, rotationApplied);
	}

	private void repulsorAutoAlign(Pose2d pose, SwerveSample sample) {
		rotationController.enableContinuousInput(MIN_HEADING_ANGLE.in(Degrees), MAX_HEADING_ANGLE.in(Degrees));

		var targetSpeeds = sample.getChassisSpeeds();
		targetSpeeds.vxMetersPerSecond += translationController.calculate(pose.getX(), sample.x);
		targetSpeeds.vyMetersPerSecond += translationController.calculate(pose.getY(), sample.y);
		targetSpeeds.omegaRadiansPerSecond += rotationController.calculate(pose.getRotation().getRadians(), sample.heading);

		drive.driveFieldRelative(-targetSpeeds.vxMetersPerSecond, -targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond);
	}

	private boolean checkForReefCollision() {
		Pose2d currentPose = drive.getPose();

		double t = calculateClosestPoint(currentPose, targetPose);
		interpolatedPose = currentPose.interpolate(targetPose, t);

		interpolatedDistanceFromReef = interpolatedPose.getTranslation().getDistance(reefPose.getTranslation());
		return interpolatedDistanceFromReef < REEF_HITBOX.in(Meters) + ROBOT_RADIUS.in(Meters);
	}

	private double calculateClosestPoint(Pose2d startPose, Pose2d endPose) {
		double numeratorX = (reefPose.getX() - startPose.getX()) * (endPose.getX() - startPose.getX());
		double numeratorY = (reefPose.getY() - startPose.getY()) * (endPose.getY() - startPose.getY());
		double denominator = Math.pow(endPose.getX() - startPose.getX(), 2) + Math.pow(endPose.getY() - startPose.getY(), 2);

		return MathUtil.clamp((numeratorX + numeratorY) / denominator, 0, 1);
	}

	private void logOutput() {
		Logger.recordOutput("AutoAlign/State", getState().getStateString());
		Logger.recordOutput("AutoAlign/Target Pose", targetPose);
		Logger.recordOutput("AutoAlign/Interpolated Pose", interpolatedPose);
		Logger.recordOutput("AutoAlign/Interpolated Distance From Reef", interpolatedDistanceFromReef);
		Logger.recordOutput("AutoAlign/ReefPose", reefPose);
		Logger.recordOutput("AutoAlign/Repulsor Activated", repulsorActivated);
	}

	public boolean nearTarget() {
		return (drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < DISTANCE_ERROR_MARGIN && Math.abs(drive.getPose().getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < ANGLE_ERROR_MARGIN);
	}

	public boolean readyForClose() {
		return (drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < getState().getDistanceForCloseAA().in(Meters));
	}
}
