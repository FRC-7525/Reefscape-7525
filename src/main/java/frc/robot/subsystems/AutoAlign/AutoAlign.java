package frc.robot.Subsystems.AutoAlign;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.RunnableTrigger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Utils.RepulsorFieldPlanner;

public class AutoAlign extends Subsystem<AutoAlignStates>{

    private static AutoAlign instance;

    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final Drive drive = Drive.getInstance();
    private final Manager manager = Manager.getInstance();

    private RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();

    private PIDController translationController;
    private PIDController rotationController;
    private Pose2d targetPose;

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

                translationController = new PIDController(1, 0, 0);
                rotationController = new PIDController(1, 0,0);
                break;  
        }

        targetPose = new Pose2d(5.6, 1.67, new Rotation2d());

        addTrigger(AutoAlignStates.IDLE, AutoAlignStates.DRIVING_REEF_L1, Controllers.DRIVER_CONTROLLER::getAButtonPressed);
    }

    @Override
    protected void runState() {
        if (getState() == AutoAlignStates.IDLE) return;

        manager.setState(getState().getManagerState());
        // braindeadAutoAlign();

        repulsor.setGoal(targetPose.getTranslation());  

        repulsorAutoAlign(drive.getPose(), repulsor.getCmd(drive.getPose(), drive.getRobotRelativeSpeeds(), 4, true, targetPose.getRotation()));

        logOutput();
    }

    public static AutoAlign getInstance() {
        if (instance == null) {
            instance = new AutoAlign();
        }
        return instance;
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
        Logger.recordOutput("AutoAlign/ChassisSpeed", targetSpeeds);
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

    private void logOutput() {
        Logger.recordOutput("AutoAlign/State", getState().getStateString());
        Logger.recordOutput("AutoAlign/Target Pose", targetPose);
    }
}
