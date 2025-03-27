package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import static frc.robot.Subsystems.Drive.TunerConstants.kSpeedAt12Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FaultManager.FaultManager;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Drive.DriveIOInputsAutoLogged;
import frc.robot.Subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {

	private static Drive instance;

	private FaultManager faultManager = FaultManager.getInstance();

	private DriveIO driveIO;
	private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
	private boolean robotMirrored = false;
	private Pose2d lastPose = new Pose2d();
	private double lastTime = 0;
	private Angle lastHeading;
	private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
	private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
	private final PIDController headingCorrectionController = new PIDController(0.1, 0, 0);
	private final Field2d field = new Field2d();
	private final SlewRateLimiter xTranslationLimiter = new SlewRateLimiter(MAX_LINEAR_DECELERATION.in(MetersPerSecondPerSecond));
	private final SlewRateLimiter yTranslationLimiter = new SlewRateLimiter(MAX_LINEAR_DECELERATION.in(MetersPerSecondPerSecond));

	private final SlewRateLimiter xElevatorUpTranslationLimiter = new SlewRateLimiter(MAX_ELEVATOR_UP_ACCEL.in(MetersPerSecondPerSecond));
	private final SlewRateLimiter yElevatorUpTranslationLimiter = new SlewRateLimiter(MAX_ELEVATOR_UP_ACCEL.in(MetersPerSecondPerSecond));

	private final SlewRateLimiter xStoppingTranslationLimiter = new SlewRateLimiter(MAX_LINEAR_STOPPING_ACCELERATION.in(MetersPerSecondPerSecond));
	private final SlewRateLimiter yStoppingTranslationLimiter = new SlewRateLimiter(MAX_LINEAR_STOPPING_ACCELERATION.in(MetersPerSecondPerSecond));

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
		};

		//Add Devices to Fault Manager
		faultManager.addDevice(driveIO.getDrive().getPigeon2(), "Pigeon 2", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(0).getDriveMotor(), "Front Left Drive Kraken", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(0).getSteerMotor(), "Front Left Turn Falcon", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(0).getEncoder(), "Front Left CANcoder", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(1).getDriveMotor(), "Front Right Drive Kraken", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(1).getSteerMotor(), "Front Right Turn Falcon", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(1).getEncoder(), "Front Right CANcoder", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(2).getDriveMotor(), "Back Left Drive Kraken", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(2).getSteerMotor(), "Back Left Turn Falcon", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(2).getEncoder(), "Back Left CANcoder", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(3).getDriveMotor(), "Back Right Drive Kraken", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(3).getSteerMotor(), "Back Right Turn Falcon", "CANivore");
		faultManager.addDevice(driveIO.getDrive().getModule(3).getEncoder(), "Back Right CANcoder", "CANivore");

		// Zero Gyro
		addRunnableTrigger(
			() -> {
				driveIO.zeroGyro();
			},
			DRIVER_CONTROLLER::getStartButtonPressed
		);

		// Wheel Radius Characterization (no I didn't test it in sim, banks said it works)
		addRunnableTrigger(
			() -> {
				CommandScheduler.getInstance().cancelAll();
				CommandScheduler.getInstance().schedule(WheelRadiusCharacterization.getInstance().getWheelRadiusCharacterizationCommand(-1, this));
			},
			TEST_CONTROLLER::getBButtonPressed
		);

		this.lastHeading = Degrees.of(driveIO.getDrive().getState().Pose.getRotation().getDegrees());
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

		if (DriverStation.isDisabled()) robotMirrored = false;

		// Zero on init/when first disabled
		if (!robotMirrored && !DriverStation.isDisabled()) {
			DriverStation.getAlliance()
				.ifPresent(allianceColor -> {
					driveIO.getDrive().setOperatorPerspectiveForward(allianceColor == Alliance.Red ? RED_ALLIANCE_PERSPECTIVE_ROTATION : BLUE_ALLIANCE_PERSPECTIVE_ROTATION);
					robotMirrored = true;
				});
		}
		logOutputs(driveIO.getDrive().getState());

		// Otherwise it will try to force wheels to stop in auto
		if (AutoAlign.getInstance().getState() == AutoAlignStates.OFF && !WheelRadiusCharacterization.getInstance().isCharacterizationActive()) {
			getState().driveRobot();
		}

		field.setRobotPose(getPose());
		SmartDashboard.putData("Field", field);
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
		FIELD.setRobotPose(lastPose);

		lastPose = state.Pose;
		lastTime = Utils.getSystemTimeSeconds();
	}

	/**
	 * Drives the robot in field-relative mode.
	 *
	 * @param xVelocity       The desired x-axis velocity.
	 * @param yVelocity       The desired y-axis velocity.
	 * @param angularVelocity The desired angular velocity.
	 * @param useHeadingCorrection Whether to use the heading correction controller.
	 */
	public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity, boolean useHeadingCorrection, boolean useDecelerationLimit) {
		//Reduces angular velocity if elevator is above certain height
		//TODO: Needs to be tuned
		final double height = Elevator.getInstance().getHeight().in(Inches);
		double scaleFactor = ((ElevatorConstants.L3_HEIGHT.in(Inches) - height + (ElevatorConstants.L4_HEIGHT.in(Inches) - ElevatorConstants.L3_HEIGHT.in(Inches)) * ANGULAR_VELOCITY_SCALE_FACTOR)) / (ElevatorConstants.L4_HEIGHT.in(Inches) - ElevatorConstants.L3_HEIGHT.in(Inches) * ANGULAR_VELOCITY_SCALE_FACTOR);

		if (scaleFactor > MIN_SCALE_FACTOR) scaleFactor = MIN_SCALE_FACTOR;

		angularVelocity *= scaleFactor;

		double omega = angularVelocity;
		if (useHeadingCorrection) {
			if (Math.abs(omega) == 0.0 && (Math.abs(xVelocity) > DEADBAND || Math.abs(yVelocity) > DEADBAND)) {
				omega = headingCorrectionController.calculate(driveIO.getDrive().getState().Pose.getRotation().getRadians(), lastHeading.in(Radians)) * 0.1 * ANGULAR_VELOCITY_LIMIT.in(RadiansPerSecond);
			} else {
				lastHeading = Degrees.of(driveIO.getDrive().getState().Pose.getRotation().getDegrees());
			}
		}

		double antiTipX = xVelocity;
		double antiTipY = yVelocity;

		if (useDecelerationLimit && !DriverStation.isAutonomous()) {
			double currentVelocity = Drive.getInstance().getVelocity().in(MetersPerSecond);
			double targetVelocity = Math.hypot(xVelocity, yVelocity);
			// Like yknow when it tips but like it be tipping mad when u stop, yeah this stops it
			// if (Math.abs(currentVelocity) > TIPPING_LIMITER_THRESHOLD.in(MetersPerSecond) && Math.abs(targetVelocity) <= 0.5 && (Elevator.getInstance().getState() == ElevatorStates.CORAL_STATION || Elevator.getInstance().getState() == ElevatorStates.IDLE)) {
			// 	Angle angle = Radians.of(Math.atan2(yVelocity, xVelocity));
			// 	antiTipX = xStoppingTranslationLimiter.calculate(targetVelocity * Math.sin(angle.in(Radians)));
			// 	antiTipY = yStoppingTranslationLimiter.calculate(targetVelocity * Math.cos(angle.in(Radians)));
			// 	Logger.recordOutput(SUBSYSTEM_NAME + "/AntiTipApplied", true);
			// } else {
			// 	// When ur tryna anti tip but you wouldn't tip anyways
			// 	if (Elevator.getInstance().getState() != ElevatorStates.CORAL_STATION || Elevator.getInstance().getState() != ElevatorStates.IDLE) {
			// 		antiTipX = xTranslationLimiter.calculate(xVelocity);
			// 		antiTipY = yTranslationLimiter.calculate(yVelocity);
			// 	} else {
			// 		antiTipX = xElevatorUpTranslationLimiter.calculate(xVelocity);
			// 		antiTipY = yElevatorUpTranslationLimiter.calculate(yVelocity);
			// 	}
			// 	Logger.recordOutput(SUBSYSTEM_NAME + "/AntiTipApplied", false);
			// }
		}

		driveIO.setControl(
			new SwerveRequest.FieldCentric()
				.withDeadband(DEADBAND)
				.withVelocityX(useDecelerationLimit && !DriverStation.isAutonomous() ? antiTipX : xVelocity)
				.withVelocityY(useDecelerationLimit && !DriverStation.isAutonomous() ? antiTipY : yVelocity)
				.withRotationalRate(omega)
				.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
				.withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
		);
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

	public void zeroGyro() {
		driveIO.zeroGyro();
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

	public Pigeon2 getPigeon2() {
		return driveIO.getDrive().getPigeon2();
	}

	public TalonFX[] getDriveMotors() {
		return driveIO.getDriveMotors();
	}

	public TalonFX[] getSteerMotors() {
		return driveIO.getSteerMotors();
	}

	public TunerSwerveDrivetrain getDriveTrain() {
		return driveIO.getDrive();
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

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		if (ROBOT_MODE == RobotMode.REAL) {
			driveIO.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp), visionMeasurementStdDevs);
		} else {
			driveIO.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
	}
}
