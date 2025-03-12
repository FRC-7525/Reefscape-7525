package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.AutoAlign.AutoAlignConstants.*;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.team7525.autoAlign.RepulsorFieldPlanner;
import org.team7525.subsystem.Subsystem;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Subsystems.Drive.Drive;

public class AutoAlign extends Subsystem<AutoAlignStates> {
    // Note for reviewer - ima put everything in constants once I finalize AA, this is a rough draft expect 200 more LOC
    
    private static AutoAlign instance;

    private final Drive drive = Drive.getInstance();
    
    private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner(
        new ArrayList<>(), 
        new ArrayList<>(), 
        (ROBOT_MODE == RobotMode.SIM)
    );

    private final ProfiledPIDController rotationController;
    private final ProfiledPIDController translationalController;
    
    private PIDController repulsorTranslationController;
    private PIDController repulsorRotationalController;
    
    private Debouncer autoAlignDebouncer;
    
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 1.0;
    
    private Pose2d targetPose;
    private Pose2d goalPose;
    private Pose2d reefPose = REEF_POSE;
    private Pose2d interpolatedPose;
    private boolean isRedAlliance;
    private double interpolatedDistanceFromReef;
    private boolean enteredBrainDead;
    private boolean repulsorActivated;
    private double timer = -1;
    
    private AutoAlign() {
        super("AutoAlign", AutoAlignStates.OFF);
        
        // FF impl (if 254 has magic number so can I hehehe)
        this.rotationController = new ProfiledPIDController(
            6, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0), 
            0.02
        );
        
        this.translationalController = new ProfiledPIDController(
            4, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0), 
            0.02
        );
        
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
        this.repulsorRotationalController.enableContinuousInput(
            MIN_HEADING_ANGLE.in(Radians), 
            MAX_HEADING_ANGLE.in(Radians)
        );
        
        this.lastSetpointTranslation = new Translation2d();
        this.autoAlignDebouncer = new Debouncer(0.5, DebounceType.kRising);
        this.repulsorActivated = false;
        this.enteredBrainDead = false;
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
        if (getState() == AutoAlignStates.OFF) return;
        
        // Update alliance and reef position
        isRedAlliance = DriverStation.getAlliance().isPresent() && 
                        DriverStation.getAlliance().get() == Alliance.Red;
        
        reefPose = isRedAlliance ? 
                  new Pose2d(13.08, 4, new Rotation2d()) : 
                  new Pose2d(4.49, 4, new Rotation2d());
        
        // Get the target pose from the current state
        goalPose = getState().getTargetPose();
        targetPose = goalPose;
        
        // Update current pose information
        Pose2d currentPose = drive.getPose();
        
        // Special handling if not ready for close-range alignment
        if (!readyForClose()) {
            Translation2d temp = reefPose.getTranslation().minus(currentPose.getTranslation());
            targetPose = new Pose2d(
                targetPose.getTranslation(), 
                Rotation2d.fromRadians(Math.atan2(temp.getY(), temp.getX()))
            );
        }
        
        // Check for collision risk with the reef
        if (!checkForReefCollision()) {
            repulsorActivated = false;
            executeScaledFeedforwardAutoAlign(currentPose);
        } else {
            repulsorActivated = true;
            executeRepulsorAutoAlign(currentPose);
        }
        
        logOutput();
    }
    
    // ACTUALLY DRIVING

    // 254's implementation of the braindead auto-align
    private void executeScaledFeedforwardAutoAlign(Pose2d currentPose) {
        // Apply scalar drive with feedforward
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = MathUtil.clamp(
            (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 
            0.0, 
            1.0
        );
        
        driveErrorAbs = currentDistance;
        translationalController.reset(
            lastSetpointTranslation.getDistance(targetPose.getTranslation()), 
            translationalController.getSetpoint().velocity
        );

        // Calculate translation velocity scalar with PID and FF scaling
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

        // Calculate rotation velocity with PID and FF scaling
        double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler
            + rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());   
        if (thetaErrorAbs < rotationController.getPositionTolerance()) {
            thetaVelocity = 0;
        }
        
        // Calculate final translation velocity
        var translationVelocity = 
            MathHelpers.pose2dFromRotation(
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                    MathHelpers.transform2dFromTranslation(
                        new Translation2d(translationVelocityScalar, 0.0)))
                .getTranslation();
       
        // Apply drive commands with alliance compensation
        if (isRedAlliance) {
            drive.driveFieldRelative(
                -translationVelocity.getX(), 
                -translationVelocity.getY(), 
                thetaVelocity, 
                false, 
                false
            );
        } else {
            drive.driveFieldRelative(
                translationVelocity.getX(), 
                translationVelocity.getY(), 
                thetaVelocity, 
                false, 
                false
            );
        }
    }
    
    private void executeRepulsorAutoAlign(Pose2d currentPose) {
        // Set repulsor goal and get command
        repulsor.setGoal(targetPose.getTranslation());
        SwerveSample sample = repulsor.getCmd(
            currentPose, 
            drive.getRobotRelativeSpeeds(), 
            MAX_SPEED.in(MetersPerSecond), 
            USE_GOAL, 
            targetPose.getRotation()
        );
        
        // Extract and modify chassis speeds with additional control
        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += repulsorTranslationController.calculate(
            currentPose.getX(), 
            sample.x
        );
        targetSpeeds.vyMetersPerSecond += repulsorTranslationController.calculate(
            currentPose.getY(), 
            sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += repulsorRotationalController.calculate(
            currentPose.getRotation().getRadians(), 
            sample.heading
        );
        
        // No more race conditions :Sob:
        if (isRedAlliance) {
            drive.driveFieldRelative(
                -targetSpeeds.vxMetersPerSecond, 
                -targetSpeeds.vyMetersPerSecond, 
                targetSpeeds.omegaRadiansPerSecond, 
                false, 
                false
            );
        } else {
            drive.driveFieldRelative(
                targetSpeeds.vxMetersPerSecond, 
                targetSpeeds.vyMetersPerSecond, 
                targetSpeeds.omegaRadiansPerSecond, 
                false, 
                false
            );
        }
    }
    

    // UTIL STUFF

    // LOS
    // TODO: Profile this to see how resource intensive it is
    private boolean checkForReefCollision() {
        Pose2d currentPose = drive.getPose();

        double t = calculateClosestPoint(currentPose, targetPose);
        interpolatedPose = currentPose.interpolate(targetPose, t);

        interpolatedDistanceFromReef = interpolatedPose.getTranslation().getDistance(reefPose.getTranslation());

        boolean willCollide = interpolatedDistanceFromReef < (REEF_HITBOX.in(Meters) + ROBOT_RADIUS.in(Meters));
        Logger.recordOutput("AutoAlign/Gona hit reef", willCollide);
        return willCollide && !(Math.abs(targetPose.getTranslation().getDistance(currentPose.getTranslation())) < 0.3);
    }

    // Used for LOS
    private double calculateClosestPoint(Pose2d startPose, Pose2d endPose) {
        double numeratorX = (reefPose.getX() - startPose.getX()) * (endPose.getX() - startPose.getX());
        double numeratorY = (reefPose.getY() - startPose.getY()) * (endPose.getY() - startPose.getY());
        double denominator = Math.pow(endPose.getX() - startPose.getX(), 2) + 
                            Math.pow(endPose.getY() - startPose.getY(), 2);

        return MathUtil.clamp((numeratorX + numeratorY) / denominator, 0, 1);
    }
    
    // Near setpoint for final transition
    public boolean nearGoal() {
        return drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < 
                DISTANCE_ERROR_MARGIN.in(Meters) && 
                (Math.abs(repulsorRotationalController.getError()) < ANGLE_ERROR_MARGIN.in(Radians) || 
                Math.abs(rotationController.getPositionError()) < ANGLE_ERROR_MARGIN.in(Radians));
    }

    // For close transitions (i.e. ready for elevator to go up)
    public boolean readyForClose() {
		return (drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < getState().getDistanceForCloseAA().in(Meters));
	}

    // IDK is this works lmfao
    public boolean nearGoalDebounced() {
        return autoAlignDebouncer.calculate(
            drive.getPose().getTranslation().getDistance(goalPose.getTranslation()) < 
                DISTANCE_ERROR_MARGIN.in(Meters) && 
                (Math.abs(repulsorRotationalController.getError()) < ANGLE_ERROR_MARGIN.in(Radians) || 
                Math.abs(rotationController.getPositionError()) < ANGLE_ERROR_MARGIN.in(Radians))
        );
    }
    
    // We never really tested ts
    public boolean timedOut() {
        if (getStateTime() > 0.01 && 
            drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < MOVEMENT_THRESHOLD) {
            timer = getStateTime();
        }
        
        if (timer != -1 && getStateTime() - timer > TIMEOUT_THRESHOLD) {
            timer = -1;
            return true;
        } else {
            return false;
        }
    }
    

    // TODO: Call this on entrance/exit & impl 254s thingy
    public void resetPID() {
        Pose2d currentPose = drive.getPose();
        ChassisSpeeds currentSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getRobotRelativeSpeeds(), 
            currentPose.getRotation()
        );

        rotationController.reset(currentSpeed.omegaRadiansPerSecond);
    }

    private void logOutput() {
        if (!Robot.isReal()) {
            Pose2d[] arrowsArray = new Pose2d[] {};
            Logger.recordOutput("AutoAlign/Arros", arrowsArray);
        }
        Logger.recordOutput("AutoAlgin/UsingRepulsor", repulsorActivated);
        Logger.recordOutput("AutoAlign/InterpolatedDistanceFromReef", interpolatedDistanceFromReef);
        Logger.recordOutput("AutoAlign/InterpolatedPose", interpolatedPose);
        Logger.recordOutput("AutoAlign/TargetPose", targetPose);
        Logger.recordOutput("AutoAlign/GoalPose", goalPose);
        Logger.recordOutput("AutoAlign/IsRedAlliance", isRedAlliance);
        Logger.recordOutput("AutoAlign/EnteredBrainDead", enteredBrainDead);
        Logger.recordOutput("AutoAlign/DriveErrorAbs", driveErrorAbs);
        Logger.recordOutput("AutoAlign/ThetaErrorAbs", thetaErrorAbs);
    }
}