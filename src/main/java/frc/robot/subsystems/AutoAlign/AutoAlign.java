package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.AutoAlign.AutoAlignConstants.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems.AutoAlign.AutoAlignConstants.Real;
import frc.robot.Subsystems.AutoAlign.AutoAlignConstants.Sim;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Utils.RepulsorFieldPlanner;
import org.littletonrobotics.junction.Logger;
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
	private Pose2d reefPose;
	private Pose2d interpolatedPose;
	private double interpolatedDistanceFromReef;
	private boolean repulsorActivated;

	private PoseContainer poseContainer;
	private BranchLevel branchLevel;
	private int reefSideNum;
	private boolean leftSideSelected;

	private static enum BranchLevel {
		L1,
		L2,
		L3,
		L4,
	}

	private AutoAlign() {
		super("AutoAlign", AutoAlignStates.OFF);
		// TODO tune real once robot is done. sim tuning is also kinda mid
		switch (ROBOT_MODE) {
			case REAL:
			case TESTING:
				translationController = new PIDController(
					Real.TRANSLATIONAL_PID_CONSTANTS.kP,
					Real.TRANSLATIONAL_PID_CONSTANTS.kI,
					Real.TRANSLATIONAL_PID_CONSTANTS.kD
				);
				rotationController = new PIDController(
					Real.ROTATIONAL_PID_CONSTANTS.kP,
					Real.ROTATIONAL_PID_CONSTANTS.kI,
					Real.ROTATIONAL_PID_CONSTANTS.kD
				);
				repulsionTranslationController = new PIDController(
					Real.TRANSLATIONAL_PID_CONSTANTS.kP,
					Real.TRANSLATIONAL_PID_CONSTANTS.kI,
					Real.TRANSLATIONAL_PID_CONSTANTS.kD
				);
				repulsionRotationController = new PIDController(
					Real.ROTATIONAL_PID_CONSTANTS.kP,
					Real.ROTATIONAL_PID_CONSTANTS.kI,
					Real.ROTATIONAL_PID_CONSTANTS.kD
				);
				poseContainer = DriverStation.getAlliance().get() == Alliance.Blue
					? BLUE_POSES
					: RED_POSES;
				break;
			case SIM:
			case REPLAY:
				translationController = new PIDController(
					Sim.TRANSLATIONAL_PID_CONSTANTS.kP,
					Sim.TRANSLATIONAL_PID_CONSTANTS.kI,
					Sim.TRANSLATIONAL_PID_CONSTANTS.kD
				);
				rotationController = new PIDController(
					Sim.ROTATIONAL_PID_CONSTANTS.kP,
					Sim.ROTATIONAL_PID_CONSTANTS.kI,
					Sim.ROTATIONAL_PID_CONSTANTS.kD
				);
				repulsionTranslationController = new PIDController(
					Sim.TRANSLATIONAL_PID_CONSTANTS.kP,
					Sim.TRANSLATIONAL_PID_CONSTANTS.kI,
					Sim.TRANSLATIONAL_PID_CONSTANTS.kD
				);
				repulsionRotationController = new PIDController(
					Sim.ROTATIONAL_PID_CONSTANTS.kP,
					Sim.ROTATIONAL_PID_CONSTANTS.kI,
					Sim.ROTATIONAL_PID_CONSTANTS.kD
				);
				poseContainer = BLUE_POSES;
				break;
		}

		targetPose = Testing.test1; // testing
		branchLevel = BranchLevel.L4;
		reefSideNum = 0;
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

		// if there is no collision, it will go to braindead AA, or use repulsor to avoid the collision
		if (!checkForReefCollision()) {
			braindeadAutoAlign();
			repulsorActivated = false;
		} else {
			repulsor.setGoal(targetPose.getTranslation());
			repulsorAutoAlign(
				drive.getPose(),
				repulsor.getCmd(
					drive.getPose(),
					drive.getRobotRelativeSpeeds(),
					MAX_SPEED,
					USE_GOAL,
					targetPose.getRotation()
				)
			);
			repulsorActivated = true;
		}
	}

	private void braindeadAutoAlign() {
		rotationController.enableContinuousInput(MIN_HEADING_ANGLE, MAX_HEADING_ANGLE);
		Pose2d drivePose = drive.getPose();

		// idk why applied needs to be negative but it works if it is negative 💀
		// update: it's negative because otto is a bum and can't set up his sim properly
		double xApplied = -repulsionTranslationController.calculate(
			drivePose.getX(),
			targetPose.getX()
		);
		double yApplied = -repulsionTranslationController.calculate(
			drivePose.getY(),
			targetPose.getY()
		);
		double rotationApplied = repulsionRotationController.calculate(
			drivePose.getRotation().getDegrees(),
			targetPose.getRotation().getDegrees()
		);
		drive.driveFieldRelative(xApplied, yApplied, rotationApplied);
	}

	// may have to use seperate PID controllers for this function instead of the one for regular AA
	// also stolen lol
	private void repulsorAutoAlign(Pose2d pose, SwerveSample sample) {
		rotationController.enableContinuousInput(MIN_HEADING_ANGLE, MAX_HEADING_ANGLE);

		var targetSpeeds = sample.getChassisSpeeds();
		targetSpeeds.vxMetersPerSecond += translationController.calculate(pose.getX(), sample.x);
		targetSpeeds.vyMetersPerSecond += translationController.calculate(pose.getY(), sample.y);
		targetSpeeds.omegaRadiansPerSecond += rotationController.calculate(
			pose.getRotation().getRadians(),
			sample.heading
		);

		drive.driveFieldRelative(
			-targetSpeeds.vxMetersPerSecond,
			-targetSpeeds.vyMetersPerSecond,
			targetSpeeds.omegaRadiansPerSecond
		);
	}

	private boolean checkForReefCollision() {
		Pose2d currentPose = drive.getPose();

		double t = calculateClosestPoint(currentPose, targetPose);
		interpolatedPose = currentPose.interpolate(targetPose, t);

		interpolatedDistanceFromReef = interpolatedPose
			.getTranslation()
			.getDistance(reefPose.getTranslation());
		return interpolatedDistanceFromReef < REEF_HITBOX.in(Meters) + ROBOT_RADIUS.in(Meters);
	}

	private double calculateClosestPoint(Pose2d startPose, Pose2d endPose) {
		double numeratorX =
			(reefPose.getX() - startPose.getX()) * (endPose.getX() - startPose.getX());
		double numeratorY =
			(reefPose.getY() - startPose.getY()) * (endPose.getY() - startPose.getY());
		double denominator =
			Math.pow(endPose.getX() - startPose.getX(), TWO) +
			Math.pow(endPose.getY() - startPose.getY(), TWO);

		return MathUtil.clamp((numeratorX + numeratorY) / denominator, ZERO, ONE);
	}

	private void logOutput() {
		Logger.recordOutput("AutoAlign/State", getState().getStateString());
		Logger.recordOutput("AutoAlign/Target Pose", targetPose);
		Logger.recordOutput("AutoAlign/Interpolated Pose", interpolatedPose);
		Logger.recordOutput(
			"AutoAlign/Interpolated Distance From Reef",
			interpolatedDistanceFromReef
		);
		Logger.recordOutput("AutoAlign/ReefPose", reefPose);
		Logger.recordOutput("AutoAlign/Repulsor Activated", repulsorActivated);
	}

	public boolean nearTarget() {
		return (
			drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) <
				DISTANCE_ERROR_MARGIN &&
			Math.abs(
				drive.getPose().getRotation().getDegrees() - targetPose.getRotation().getDegrees()
			) <
			ANGLE_ERROR_MARGIN
		);
	}

	// TODO: This
	public boolean readyForClose() {
		return false;
	}

	// TODO change these once we get the CAD
	// TODO this is also kinda buns
	private void launchAutoAlign() {
		targetPose = leftSideSelected
			? poseContainer.getReefSides(reefSideNum).leftPose
			: poseContainer.getReefSides(reefSideNum).rightPose;

		// switch (branchLevel) {
		// 	case L1:
		// 		setState(AutoAlignStates.DRIVING_REEF);
		// 		break;
		// 	case L2:
		// 		setState(AutoAlignStates.DRIVING_REEF);
		// 		break;
		// 	case L3:
		// 		setState(AutoAlignStates.DRIVING_REEF);
		// 		break;
		// 	case L4:
		// 		setState(AutoAlignStates.);
		// 		break;
		// }
	}
}
