package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import static frc.robot.Subsystems.Drive.TunerConstants.kSpeedAt12Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {

	private static Drive instance;

	private DriveIO driveIO;
	private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
	private boolean robotMirrored = false;
	private Pose2d lastPose = new Pose2d();
	private double lastTime = 0;
	private final Field2d field = new Field2d();

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
		if (AutoAlign.getInstance().getState() == AutoAlignStates.OFF) {
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

	public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
		//Reduces angular velocity if elevator is above certain height
		//TODO: Needs to be tuned
		final double height = Elevator.getInstance().getHeight().in(Inches);
		double scaleFactor =
			((ElevatorConstants.L3_HEIGHT.in(Inches) - height + (ElevatorConstants.L4_HEIGHT.in(Inches) - ElevatorConstants.L3_HEIGHT.in(Inches)) * ANGULAR_VELOCITY_SCALE_FACTOR)) / (ElevatorConstants.L4_HEIGHT.in(Inches) - ElevatorConstants.L3_HEIGHT.in(Inches) * ANGULAR_VELOCITY_SCALE_FACTOR);

		if (scaleFactor > 1) scaleFactor = 1;
		if (scaleFactor < MIN_SCALE_FACTOR) scaleFactor = MIN_SCALE_FACTOR;

		angularVelocity *= scaleFactor;

		driveIO.setControl(
			new SwerveRequest.FieldCentric()
				.withDeadband(DEADBAND)
				.withVelocityX(xVelocity)
				.withVelocityY(yVelocity)
				.withRotationalRate(angularVelocity)
				.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
				.withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
		);
	}

	
	public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
		driveIO.setControl(new SwerveRequest.RobotCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
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

	// Util
	public Pose2d getPose() {
		return driveIO.getDrive().getState().Pose;
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return driveIO.getDrive().getState().Speeds;
	}

	public LinearVelocity getVelocity() {
		return MetersPerSecond.of(Math.hypot(driveIO.getDrive().getState().Speeds.vxMetersPerSecond, driveIO.getDrive().getState().Speeds.vyMetersPerSecond));
	}

	public TunerSwerveDrivetrain getDriveTrain() {
		return driveIO.getDrive();
	}

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		if (ROBOT_MODE == RobotMode.REAL) {
			driveIO.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp), visionMeasurementStdDevs);
		} else {
			driveIO.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
	}
}
