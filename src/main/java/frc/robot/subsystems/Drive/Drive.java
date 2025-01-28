package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import static frc.robot.Subsystems.Drive.TunerConstants.kSpeedAt12Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {

	private static Drive instance;

	private DriveIO driveIO;
	private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
	private boolean robotMirrored = false;
	private Pose2d lastPose = new Pose2d();
	private double lastTime = 0;
	private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
	private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

	/**
	 * Constructs a new Drive subsystem with the given DriveIO.
	 *
	 * @param driveIO The DriveIO object used for controlling the drive system.
	 */
	private Drive() {
		super("Drive", DriveStates.FIELD_RELATIVE);
		this.driveIO = switch (ROBOT_MODE) {
			case REAL -> new DriveIOReal();
			case SIM -> new DriveIOSim();
			case TESTING -> new DriveIOReal();
			case REPLAY -> new DriveIOSim();
		};

		// Setup Path Planner
		configurePathPlanner();

		// Zero Gyro
		addRunnableTrigger(
			() -> {
				driveIO.zeroGyro();
			},
			DRIVER_CONTROLLER::getStartButtonPressed
		);
	}

	/**
	 * Returns the singleton instance of the Drive subsystem.
	 *
	 * @return The Drive Instance.
	 */
	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	@Override
	public void runState() {
		driveIO.updateInputs(inputs);
		Logger.processInputs("Drive", inputs);

		// Zero on init/when first disabled
		if (!robotMirrored || DriverStation.isDisabled()) {
			DriverStation.getAlliance()
				.ifPresent(allianceColor -> {
					driveIO.getDrive().setOperatorPerspectiveForward(allianceColor == Alliance.Red ? RED_ALLIANCE_PERSPECTIVE_ROTATION : BLUE_ALLIANCE_PERSPECTIVE_ROTATION);
					robotMirrored = true;
				});
		}
		logOutputs(driveIO.getDrive().getState());

		// Otherwise it will try to force wheels to stop in auto
		if (!DriverStation.isAutonomous() && AutoAlign.getInstance().getState() == AutoAlignStates.OFF) {
			getState().driveRobot();
		}
	}

	/**
	 * Logs the outputs of the drive system.
	 *
	 * @param state The current state of the SwerveDrive.
	 */
	public void logOutputs(SwerveDriveState state) {
		Logger.recordOutput(SUBSYSTEM_NAME + "/Robot Pose", state.Pose);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Current Time", Utils.getSystemTimeSeconds());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Chassis Speeds", state.Speeds);
		Logger.recordOutput(SUBSYSTEM_NAME + "/velocity", Units.metersToFeet(Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)));
		Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModuleStates", state.ModuleStates);
		Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModulePosition", state.ModulePositions);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Translation Difference", state.Pose.getTranslation().minus(lastPose.getTranslation()));
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Pose Jumped", Math.hypot(state.Pose.getTranslation().minus(lastPose.getTranslation()).getX(), state.Pose.getTranslation().minus(lastPose.getTranslation()).getY()) > (kSpeedAt12Volts.in(MetersPerSecond) * 2 * (Utils.getSystemTimeSeconds() - lastTime)));

		lastPose = state.Pose;
		lastTime = Utils.getSystemTimeSeconds();
	}

	/**
	 * Drives the robot in field-relative mode.
	 *
	 * @param xVelocity       The desired x-axis velocity.
	 * @param yVelocity       The desired y-axis velocity.
	 * @param angularVelocity The desired angular velocity.
	 */
	public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
		driveIO.setControl(new SwerveRequest.FieldCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
	}

	/**
	 * Drives the robot in robot-relative mode.
	 *
	 * @param xVelocity       The desired x-axis velocity.
	 * @param yVelocity       The desired y-axis velocity.
	 * @param angularVelocity The desired angular velocity.
	 */
	public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
		driveIO.setControl(new SwerveRequest.RobotCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
	}

	/**
	 * Locks the wheels of the robot.
	 */
	public void lockWheels() {
		driveIO.setControl(new SwerveRequest.SwerveDriveBrake().withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
	}

	public void addVisionMeasument(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {
		driveIO.addVisionMeasurement(pose, timestamp, standardDeviaton);
	}

	// SYSId Trash (no hate ofc)
	public enum SysIdMode {
		TRANSLATION,
		STEER,
		ROTATION,
	}

	/*
	 * SysId routine for characterizing rotation.
	 * This is used to find PID gains for the FieldCentrvesicFacingAngle
	 * HeadingController.
	 * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
	 * importing the log to SysId.
	 */
	private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
		new SysIdRoutine.Config(
			/* This is in radians per second², but SysId only supports "volts per second" */
			Volts.of(Math.PI / 6).per(Second),
			/* This is in radians per second, but SysId only supports "volts" */
			Volts.of(Math.PI),
			null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			output -> {
				/* output is actually radians per second, but SysId only supports "volts" */
				driveIO.getDrive().setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
				/* also log the requested output for SysId */
				SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
			},
			null,
			this
		)
	);

	/*
	 * SysId routine for characterizing translation. This is used to find PID gains
	 * for the drive motors.
	 */
	private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
		new SysIdRoutine.Config(
			null, // Use default ramp rate (1 V/s)
			Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
			null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(output -> driveIO.getDrive().setControl(translationCharacterization.withVolts(output)), null, this)
	);

	/*
	 * SysId routine for characterizing steer. This is used to find PID gains for
	 * the steer motors.
	 */
	private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
		new SysIdRoutine.Config(
			null, // Use default ramp rate (1 V/s)
			Volts.of(7), // Use dynamic voltage of 7 V
			null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
		),
		new SysIdRoutine.Mechanism(volts -> driveIO.getDrive().setControl(steerCharacterization.withVolts(volts)), null, this)
	);

	@Override
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		switch (SYS_ID_MODE) {
			case ROTATION:
				return sysIdRoutineRotation.quasistatic(direction);
			case TRANSLATION:
				return sysIdRoutineTranslation.quasistatic(direction);
			case STEER:
				return sysIdRoutineSteer.quasistatic(direction);
			default:
				return new PrintCommand("Invalid SysId mode (Quasistatic)");
		}
	}

	@Override
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		switch (SYS_ID_MODE) {
			case ROTATION:
				return sysIdRoutineRotation.dynamic(direction);
			case TRANSLATION:
				return sysIdRoutineTranslation.dynamic(direction);
			case STEER:
				return sysIdRoutineSteer.dynamic(direction);
			default:
				return new PrintCommand("Invalid SysId mode (Dynamic)");
		}
	}

	// Path Planner UTIL

	public void driveRobotRelative(ChassisSpeeds speeds) {
		driveRobotRelative(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
	}

	// Better at driving stuff or sum (I think this makes a meh difference)
	public void driveRobotRelativeWithFF(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
		driveIO.getDrive().setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
	}

	public Pose2d getPose() {
		return driveIO.getDrive().getState().Pose;
	}

	public void resetPose(Pose2d pose) {
		driveIO.getDrive().resetPose(pose);
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return driveIO.getDrive().getState().Speeds;
	}

	public LinearVelocity getVelocity() {
		return MetersPerSecond.of(Math.hypot(driveIO.getDrive().getState().Speeds.vxMetersPerSecond, driveIO.getDrive().getState().Speeds.vyMetersPerSecond));
	}

	public void driveAutoAlign(ApplyFieldSpeeds fieldSpeeds, double[] moduleForcesX, double[] moduleForcesY) {
		driveIO
			.getDrive()
			.setControl(
				fieldSpeeds
				// .withWheelForceFeedforwardsX(moduleForcesX)
				// .withWheelForceFeedforwardsY(moduleForcesY)
			);
	}

	public void configurePathPlanner() {
		AutoBuilder.configure(
			this::getPose, // Robot pose supplier
			this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
			this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
			this::driveRobotRelativeWithFF, // Method that will drive the robot given ROBOT
			// RELATIVE ChassisSpeeds. Also optionally outputs
			// individual module feedforwards
			PATH_PLANNER_PID,
			ROBOT_CONFIG, // The robot configuration
			() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
			this // Reference to this subsystem to set requirements
		);
	}

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		driveIO.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
	}
}
