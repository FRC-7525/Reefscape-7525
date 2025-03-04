package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.AutoAlign.AutoAlignConstants.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.team7525.autoAlign.RepulsorFieldPlanner;
import org.team7525.subsystem.Subsystem;

public class AutoAlign extends Subsystem<AutoAlignStates> {

	private static AutoAlign instance;

	private final Drive drive = Drive.getInstance();
	private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner(new ArrayList<>(), new ArrayList<>(), (ROBOT_MODE == RobotMode.SIM));

	private PIDController translationXController;
	private PIDController translationYController;

	private ProfiledPIDController rotationController;

	private ProfiledPIDController repulsionTranslationController;
	private ProfiledPIDController repulsionRotationController;

	private Pose2d targetPose;
	private Pose2d goalPose;
	private Pose2d reefPose = REEF_POSE;
	private boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
	private Pose2d interpolatedPose;
	private double interpolatedDistanceFromReef;
	private boolean repulsorActivated;

	private double xApplied = 0;
	private double yApplied = 0;

	private AutoAlign() {
		super("AutoAlign", AutoAlignStates.OFF);
		// PID Config
		this.translationXController = TRANSLATIONALX_CONTROLLER.get();
		this.translationYController = TRANSLATIONALY_CONTROLLER.get();
		this.rotationController = ROTATIONAL_CONTROLLER.get();
		this.repulsionTranslationController = REPULSOR_TRANSLATIONAL_CONTROLLER.get();
		this.repulsionRotationController = REPULSOR_ROTATIONAL_CONTROLLER.get();

		repulsorActivated = false;
		targetPose = new Pose2d();
		goalPose = new Pose2d();

		this.translationXController.setTolerance(0.01);
		this.translationYController.setTolerance(0.01);
		this.repulsionRotationController.setTolerance(0.1);
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
		if (ROBOT_MODE == RobotMode.SIM) {
			reefPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? new Pose2d(13.08, 4, new Rotation2d()) : new Pose2d(4.49, 4, new Rotation2d());
			isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
		}

		if (getState() == AutoAlignStates.OFF) return;

		goalPose = getState().getTargetPose();
		targetPose = getState().getTargetPose();
		if (!readyForClose()) {
			Translation2d temp = reefPose.getTranslation().minus(drive.getPose().getTranslation());
			targetPose = new Pose2d(targetPose.getTranslation(), Rotation2d.fromRadians(Math.atan2(temp.getY(), temp.getX())));
		}

		// if there is no collision, it will go to braindead AA, or use repulsor to
		// avoid the collision
		if (!checkForReefCollision()) {
			repulsorActivated = false;
			braindeadAutoAlign();
		} else {
			repulsorActivated = true;
			repulsor.setGoal(targetPose.getTranslation());
			repulsorAutoAlign(drive.getPose(), repulsor.getCmd(drive.getPose(), drive.getRobotRelativeSpeeds(), MAX_SPEED.in(MetersPerSecond), USE_GOAL, targetPose.getRotation()));
		}
	}

	private void braindeadAutoAlign() {
		rotationController.enableContinuousInput(MIN_HEADING_ANGLE.in(Radian), MAX_HEADING_ANGLE.in(Radian));
		Pose2d drivePose = drive.getPose();

		// idk why applied needs to be negative but it works if it is negative 💀
		// update: it's negative because otto is a bum and can't set up his sim properly

		xApplied = translationXController.calculate(drivePose.getX(), targetPose.getX());
		yApplied = translationYController.calculate(drivePose.getY(), targetPose.getY());

		double rotationApplied = rotationController.calculate(drivePose.getRotation().getRadians(), targetPose.getRotation().getRadians());
		if (isRedAlliance) drive.driveFieldRelative(-xApplied, -yApplied, rotationApplied, false, false);
		else drive.driveFieldRelative(xApplied, yApplied, rotationApplied, false, false);
	}

	private void repulsorAutoAlign(Pose2d pose, SwerveSample sample) {
		repulsionRotationController.enableContinuousInput(MIN_HEADING_ANGLE.in(Radians), MAX_HEADING_ANGLE.in(Radians));

		var targetSpeeds = sample.getChassisSpeeds();
		targetSpeeds.vxMetersPerSecond += repulsionTranslationController.calculate(pose.getX(), sample.x);
		targetSpeeds.vyMetersPerSecond += repulsionTranslationController.calculate(pose.getY(), sample.y);
		targetSpeeds.omegaRadiansPerSecond += repulsionRotationController.calculate(pose.getRotation().getRadians(), sample.heading);

		Logger.recordOutput("TESTING VX", targetSpeeds.vxMetersPerSecond);
		Logger.recordOutput("TESTING VY", targetSpeeds.vyMetersPerSecond);
		if (isRedAlliance) drive.driveFieldRelative(-targetSpeeds.vxMetersPerSecond, -targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false, false);
		else drive.driveFieldRelative(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, false, false);
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
		Pose2d[] arrowsArray = new Pose2d[] {};

		Logger.recordOutput("AutoAlign/State", getState().getStateString());
		Logger.recordOutput("AutoAlign/Target Pose", targetPose);
		Logger.recordOutput("AutoAlign/Interpolated Pose", interpolatedPose);
		Logger.recordOutput("AutoAlign/Interpolated Distance From Reef", interpolatedDistanceFromReef);
		Logger.recordOutput("AutoAlign/ReefPose", reefPose);
		Logger.recordOutput("AutoAlign/Repulsor Activated", repulsorActivated);
		if (Robot.isSimulation()) {
			Logger.recordOutput("AutoAlign/Arrows", repulsor.getArrows().toArray(arrowsArray));
		}
	}

	public boolean nearGoal() {
		Logger.recordOutput("AutoTest/Rotation Error Repulsor", repulsionRotationController.getPositionError());
		Logger.recordOutput("AutoTest/Rotation Error Regular", rotationController.getPositionError());
		Logger.recordOutput("AutoTest/Translation Error", drive.getPose().getTranslation().getDistance(goalPose.getTranslation()));
		Logger.recordOutput(
			"AutoTest/NearGoal",
			drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < DISTANCE_ERROR_MARGIN && (Math.abs(repulsionRotationController.getPositionError()) < ANGLE_ERROR_MARGIN || Math.abs(rotationController.getPositionError()) < ANGLE_ERROR_MARGIN)
		);
		return (drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < DISTANCE_ERROR_MARGIN && (Math.abs(repulsionRotationController.getPositionError()) < ANGLE_ERROR_MARGIN || Math.abs(rotationController.getPositionError()) < ANGLE_ERROR_MARGIN));
	}

	public boolean readyForClose() {
		return (drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < getState().getDistanceForCloseAA().in(Meters));
	}
}
