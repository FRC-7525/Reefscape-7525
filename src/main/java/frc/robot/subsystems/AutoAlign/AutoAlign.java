package frc.robot.Subsystems.AutoAlign;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Subsystems.Drive.Drive;

public class AutoAlign extends Subsystem<AutoAlignStates> {
    // 254 code (its actually so broken omg I loveeeee 254)
    
    public static AutoAlign instance;

    private final ProfiledPIDController rotationController = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0, 0), 0.02);
    private final ProfiledPIDController translationalController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(0, 0), 0.02);

    private final Drive drive = Drive.getInstance();

    private Translation2d lastSetpointTranslation;

    private double driveErrorAbs;
    private double thetaErrorAbs;

    // 1 Meter away and more u use max ff, 0.2 meters away or less u rely on pure PID and scale down on the interval :boiled:
    private double ffMinRadius = 0.2, ffMaxRadius = 1.0;
    
    private final Supplier<Pose2d> targetPoseSupplier = getState()::getTargetPose;
    private final Supplier<Pose2d> currentPoseSupplier = drive::getPose;

    
    private AutoAlign() {
        super("AutoAlign", AutoAlignStates.OFF);

        this.lastSetpointTranslation = new Translation2d();
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static AutoAlign getInstance() {
        if (instance == null) {
            instance = new AutoAlign();
        }
        return instance;
    }

    @Override
    protected void runState() {
        if (getState() == AutoAlignStates.OFF) return;


        Pose2d targetPose = targetPoseSupplier.get();
        Pose2d currentPose = currentPoseSupplier.get();

        Logger.recordOutput("AutoAlign/Target Pose", targetPose);
        Logger.recordOutput("AutoAlign/Current Pose", currentPose);

        // Apply scalar drive around lalala
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = MathUtil.clamp(
            (currentDistance-ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        
        driveErrorAbs = currentDistance;
        translationalController.reset(lastSetpointTranslation.getDistance(targetPose.getTranslation()), translationalController.getSetpoint().velocity);

        // drive error to 0 
        double translationVelocityScalar = translationalController.getSetpoint().velocity * ffScaler
            + translationalController.calculate(driveErrorAbs, 0.0);

        if (currentDistance < translationalController.getPositionTolerance()) {
            translationVelocityScalar = 0;
        }

        lastSetpointTranslation = new Pose2d(
            targetPose.getTranslation(),
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        MathHelpers.transform2dFromTranslation(
                                new Translation2d(translationalController.getSetpoint().position, 0.0)))
                .getTranslation();

        // Spin around
        double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler
            + rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());   
        if (thetaErrorAbs < rotationController.getPositionTolerance()) {
            thetaVelocity = 0;
        }
        
        // Drive around
         var translationVelocity = 
                MathHelpers.pose2dFromRotation(currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(MathHelpers.transform2dFromTranslation(new Translation2d(translationVelocityScalar, 0.0)))
                .getTranslation();
       
        drive.driveFieldRelative(translationVelocity.getX(), translationVelocity.getY(), thetaVelocity, false, false);
    

        

        
    }
}