package frc.robot.Subsystems.AutoAlign;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.RunnableTrigger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Utils.RepulsorFieldPlanner;

public class AutoAlign extends Subsystem<AutoAlignStates>{

    private static AutoAlign instance;

    private final Drive drive = Drive.getInstance();
    private final Manager manager = Manager.getInstance();

    private RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();

    private final double ROBOT_RADIUS = 0;
    private final double REEF_RADIUS = 1.71; //1.66 m radius + 0.05 m for 
    private PIDController translationController;
    private PIDController rotationController;
    private Pose2d targetPose;
    private Pose2d reefPose = new Pose2d(4.57, 4.09, new Rotation2d());
    private Pose2d interpolatedPose;
    private double interpolatedDistanceFromReef;
    private boolean repulsorActivated;

    private AutoAlign() {
        super("AutoAlign", AutoAlignStates.IDLE);

        switch (ROBOT_MODE) {
            case REAL:
            case TESTING:
                translationController = new PIDController(1, 0, 0);
                rotationController = new PIDController(1, 0,0);
                break;
            case SIM:
            case REPLAY:
                translationController = new PIDController(2.5, 0, 0);
                rotationController = new PIDController(.8, 0,0);
                break;  
        }

        targetPose = new Pose2d(5.6, 1.67, new Rotation2d());

        addTrigger(AutoAlignStates.IDLE, AutoAlignStates.DRIVING_REEF_L1, Controllers.DRIVER_CONTROLLER::getAButtonPressed);
    }

    public static AutoAlign getInstance() {
        if (instance == null) {
            instance = new AutoAlign();
        }
        return instance;
    }

    @Override
    protected void runState() {
        if (getState() == AutoAlignStates.IDLE) return;

        manager.setState(getState().getManagerState());

        if (!checkCollision()) {
            braindeadAutoAlign();
            repulsorActivated = false;
        } else {
            repulsor.setGoal(targetPose.getTranslation());  
            repulsorAutoAlign(drive.getPose(), repulsor.getCmd(drive.getPose(), drive.getRobotRelativeSpeeds(), 4, true, targetPose.getRotation()));
            repulsorActivated = true;
        }

        logOutput();
    }


    private void braindeadAutoAlign() {
        rotationController.enableContinuousInput(-180, 180);
        Pose2d drivePose = drive.getPose();

        // idk why applied needs to be negative but it works if it is negative 💀
        double xApplied = -translationController.calculate(drivePose.getX(), targetPose.getX());
        double yApplied = -translationController.calculate(drivePose.getY(), targetPose.getY());
        double rotationApplied = rotationController.calculate(drivePose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
        drive.driveFieldRelative(xApplied, yApplied, rotationApplied);
    }

    // may have to use seperate PID controllers for this function instead of the one for regular AA
    // also stolen lol
    private void repulsorAutoAlign(Pose2d pose, SwerveSample sample) {
        rotationController.enableContinuousInput(-180, 180);

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
    private boolean checkCollision() {
        Pose2d currentPose = drive.getPose();

        double t = calculateClosestPoint(currentPose, targetPose);
        interpolatedPose = currentPose.interpolate(targetPose, t);

        interpolatedDistanceFromReef = interpolatedPose.getTranslation().getDistance(reefPose.getTranslation());
        return interpolatedDistanceFromReef < REEF_RADIUS + ROBOT_RADIUS;
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

    public boolean atTarget() {
        return drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.1;
    }
}

