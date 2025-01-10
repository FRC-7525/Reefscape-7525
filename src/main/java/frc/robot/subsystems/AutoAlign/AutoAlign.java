package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.GlobalConstants.Controllers.FIGHT_STICK;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;
import static frc.robot.Subsystems.AutoAlign.AutoAlignConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;


import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Utils.RepulsorFieldPlanner;

public class AutoAlign extends Subsystem<AutoAlignStates>{

    private static AutoAlign instance;

    private final Drive drive = Drive.getInstance();
    private final Manager manager = Manager.getInstance();

    private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();

    private PIDController translationController;
    private PIDController rotationController;
    private PIDController repulsionTranslationController;
    private PIDController repulsionRotationController;
    private Pose2d targetPose;
    private Pose2d reefPose = new Pose2d(4.57, 4.09, new Rotation2d());
    private Pose2d interpolatedPose;
    private double interpolatedDistanceFromReef;
    private boolean repulsorActivated;

    private BranchLevel branchLevel;
    private int branchNumber;
    private ReefSide reefSide;

    private static enum ReefSide {
        left,
        right
    }

    private static enum BranchLevel {
        L1,
        L2,
        L3,
        L4
    }

    private AutoAlign() {
        super("AutoAlign", AutoAlignStates.IDLE);

        // TODO tune real once robot is done. sim tuning is also kinda mid
        switch (ROBOT_MODE) {
            case REAL:
            case TESTING:
                translationController = new PIDController(Real.TRANSLATIONAL_PID_CONSTANTS.kP,Real.TRANSLATIONAL_PID_CONSTANTS.kI, Real.TRANSLATIONAL_PID_CONSTANTS.kD);
                rotationController = new PIDController(Real.ROTATIONAL_PID_CONSTANTS.kP, Real.ROTATIONAL_PID_CONSTANTS.kI, Real.ROTATIONAL_PID_CONSTANTS.kD);
                repulsionTranslationController = new PIDController(Real.TRANSLATIONAL_PID_CONSTANTS.kP,Real.TRANSLATIONAL_PID_CONSTANTS.kI, Real.TRANSLATIONAL_PID_CONSTANTS.kD);
                repulsionRotationController = new PIDController(Real.ROTATIONAL_PID_CONSTANTS.kP, Real.ROTATIONAL_PID_CONSTANTS.kI, Real.ROTATIONAL_PID_CONSTANTS.kD);
                break;
            case SIM:
            case REPLAY:
                translationController = new PIDController(Sim.TRANSLATIONAL_PID_CONSTANTS.kP,Sim.TRANSLATIONAL_PID_CONSTANTS.kI, Sim.TRANSLATIONAL_PID_CONSTANTS.kD);
                rotationController = new PIDController(Sim.ROTATIONAL_PID_CONSTANTS.kP, Sim.ROTATIONAL_PID_CONSTANTS.kI, Sim.ROTATIONAL_PID_CONSTANTS.kD);
                repulsionTranslationController = new PIDController(Sim.TRANSLATIONAL_PID_CONSTANTS.kP,Sim.TRANSLATIONAL_PID_CONSTANTS.kI, Sim.TRANSLATIONAL_PID_CONSTANTS.kD);
                repulsionRotationController = new PIDController(Sim.ROTATIONAL_PID_CONSTANTS.kP, Sim.ROTATIONAL_PID_CONSTANTS.kI, Sim.ROTATIONAL_PID_CONSTANTS.kD);
                break;  
        }

        targetPose = Poses.Testing.test1; // testing

        addRunnableTrigger(() -> branchNumber = 1, () -> FIGHT_STICK.getRawButtonPressed(3));
        addRunnableTrigger(() -> branchNumber = 2, () -> FIGHT_STICK.getRawButtonPressed(4));
        addRunnableTrigger(() -> branchNumber = 3, () -> FIGHT_STICK.getRawButtonPressed(6));
        addRunnableTrigger(() -> branchNumber = 4, () -> FIGHT_STICK.getRawButtonPressed(5));
        addRunnableTrigger(() -> branchNumber = 5, () -> FIGHT_STICK.getRawButtonPressed(1));
        addRunnableTrigger(() -> branchNumber = 6, () -> FIGHT_STICK.getRawButtonPressed(2));
        addRunnableTrigger(() -> reefSide = ReefSide.left, () -> FIGHT_STICK.getRawAxis(3) > .9);
        addRunnableTrigger(() -> reefSide = ReefSide.right, () -> FIGHT_STICK.getRawAxis(2) > .9);
        addRunnableTrigger(this::setReefLevel, () -> FIGHT_STICK.getPOV() != -1);

        addTrigger(AutoAlignStates.IDLE, AutoAlignStates.DRIVING_REEF_L1, () -> FIGHT_STICK.getPOV(0) == 270);
        addTrigger(AutoAlignStates.IDLE, AutoAlignStates.DRIVING_REEF_L2, () -> FIGHT_STICK.getPOV(0) == 180);
        addTrigger(AutoAlignStates.IDLE, AutoAlignStates.DRIVING_REEF_L3, () -> FIGHT_STICK.getPOV(0) == 90);
        addTrigger(AutoAlignStates.IDLE, AutoAlignStates.DRIVING_REEF_L4, () -> FIGHT_STICK.getPOV(0) == 0);

        addRunnableTrigger(this::setTargetPose, OPERATOR_CONTROLLER::getYButtonPressed);
        addRunnableTrigger(() -> setState(AutoAlignStates.IDLE), this::atTarget);
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

        if (getState() == AutoAlignStates.IDLE) return;

        manager.setState(getState().getManagerState());


        // if there is no collision, it will go to braindead AA, or use repulsor to avoid the collision
        if (!checkForReefCollision()) {
            braindeadAutoAlign();
            repulsorActivated = false;
        } else {
            repulsor.setGoal(targetPose.getTranslation());  
            repulsorAutoAlign(drive.getPose(), repulsor.getCmd(drive.getPose(), drive.getRobotRelativeSpeeds(), MAX_SPEED, USE_GOAL, targetPose.getRotation()));
            repulsorActivated = true;
        }

    }


    private void braindeadAutoAlign() {
        rotationController.enableContinuousInput(MIN_HEADING_ANGLE, MAX_HEADING_ANGLE);
        Pose2d drivePose = drive.getPose();

        // idk why applied needs to be negative but it works if it is negative 💀
        // update: it's negative because otto is a bum and can't set up his sim properly
        double xApplied = -repulsionTranslationController.calculate(drivePose.getX(), targetPose.getX());
        double yApplied = -repulsionTranslationController.calculate(drivePose.getY(), targetPose.getY());
        double rotationApplied = repulsionRotationController.calculate(drivePose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
        drive.driveFieldRelative(xApplied, yApplied, rotationApplied);
    }

    // may have to use seperate PID controllers for this function instead of the one for regular AA
    // also stolen lol
    private void repulsorAutoAlign(Pose2d pose, SwerveSample sample) {
        rotationController.enableContinuousInput(MIN_HEADING_ANGLE, MAX_HEADING_ANGLE);

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += translationController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += translationController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += rotationController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        drive.driveFieldRelative(-targetSpeeds.vxMetersPerSecond,
        -targetSpeeds.vyMetersPerSecond,
        targetSpeeds.omegaRadiansPerSecond);
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
        double denominator = Math.pow(endPose.getX() - startPose.getX(), TWO) + Math.pow(endPose.getY() - startPose.getY(), TWO);

        return MathUtil.clamp((numeratorX + numeratorY) / denominator, ZERO, ONE);
    }

    private void setReefLevel() {
        branchLevel = switch((int) FIGHT_STICK.getRawAxis(2)) {
            case 0 -> BranchLevel.L4;
            case 90 -> BranchLevel.L3;
            case 180 -> BranchLevel.L2;
            case 270 -> BranchLevel.L1;

            // ignore default, the fight stick will only go to multiples of 45
            default -> BranchLevel.L4;
        };
    }

    private void logOutput() {
        Logger.recordOutput("AutoAlign/State", getState().getStateString());
        Logger.recordOutput("AutoAlign/Target Pose", targetPose);
        Logger.recordOutput("AutoAlign/Interpolated Pose", interpolatedPose);
        Logger.recordOutput("AutoAlign/Interpolated Distance From Reef", interpolatedDistanceFromReef);
        Logger.recordOutput("AutoAlign/ReefPose", reefPose);
        Logger.recordOutput("AutoAlign/Repulsor Activated", repulsorActivated);
    }

    public boolean atTarget() {
        return drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < DISTANCE_ERROR_MARGIN &&
        Math.abs(drive.getPose().getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < ANGLE_ERROR_MARGIN;
    }


    // TODO change these once we get the CAD
    // TODO this is also kinda buns
    private void setTargetPose() {
       if (reefSide == ReefSide.left) {
            targetPose = switch (branchNumber) {
                case 1 -> AutoAlignConstants.Poses.Testing.test1;
                case 2 -> AutoAlignConstants.Poses.Testing.test1;
                case 3 -> AutoAlignConstants.Poses.Testing.test1;
                case 4 -> AutoAlignConstants.Poses.Testing.test1;
                case 5 -> AutoAlignConstants.Poses.Testing.test1;
                case 6 -> AutoAlignConstants.Poses.Testing.test1;
                default -> AutoAlignConstants.Poses.Testing.test1;
            };
        } else {
            targetPose = switch (branchNumber) {
                case 1 -> AutoAlignConstants.Poses.Testing.test1;
                case 2 -> AutoAlignConstants.Poses.Testing.test1;
                case 3 -> AutoAlignConstants.Poses.Testing.test1;
                case 4 -> AutoAlignConstants.Poses.Testing.test1;
                case 5 -> AutoAlignConstants.Poses.Testing.test1;
                case 6 -> AutoAlignConstants.Poses.Testing.test1;
                default -> AutoAlignConstants.Poses.Testing.test1;
            };
        }
    } 
}