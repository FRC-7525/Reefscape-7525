package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.TunerConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This class implements the DriveIO interface and provides the real implementation for controlling the drivetrain.
 */
public class DriveIOReal implements DriveIO {

	public DriveIOInputs inputs = new DriveIOInputs();
	private final TunerSwerveDrivetrain drivetrain = new TunerSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);

	/**
	 * Updates the inputs for the drivetrain based on the current state of the SwerveDrivetrain.
	 * @param inputs The DriveIOInputs object to update.
	 */
	@Override
	public void updateInputs(DriveIOInputs inputs) {
		inputs.speeds = drivetrain.getState().Speeds;
		inputs.setPoints = drivetrain.getState().ModuleTargets;
		inputs.odometryFrequency = drivetrain.getOdometryFrequency();
		inputs.timestamp = drivetrain.getState().Timestamp;
		inputs.failedDataAquisitions = drivetrain.getState().FailedDaqs;
		inputs.robotAngleDeg = drivetrain.getState().Pose.getRotation().getDegrees();
		inputs.fullRobotRotation = drivetrain.getRotation3d();
		inputs.gyroAngleDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
	}

	/**
	 * Gets the SwerveDrivetrain object.
	 * @return The SwerveDrivetrain object.
	 */
	@Override
	public TunerSwerveDrivetrain getDrive() {
		return drivetrain;
	}

	/**
	 * Resets the gyro angle to zero.
	 */
	@Override
	public void zeroGyro() {
		drivetrain.resetRotation(new Rotation2d());
	}

	/**
	 * Sets the control mode and target for the SwerveDrivetrain.
	 * @param request The SwerveRequest object containing the control mode and target.
	 */
	@Override
	public void setControl(SwerveRequest request) {
		drivetrain.setControl(request);
	}

	/**
	 * Adds a vision measurement to the SwerveDrivetrain for localization.
	 * @param pose The Pose2d object representing the measured pose.
	 * @param timestamp The timestamp of the measurement.
	 * @param standardDeviaton The standard deviation of the measurement.
	 */
	@Override
	public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {
		drivetrain.addVisionMeasurement(pose, timestamp, standardDeviaton);
		Logger.recordOutput("VisionTuning/STDev", standardDeviaton);
		Logger.recordOutput("VisionTuning/Timestamp", timestamp);
		Logger.recordOutput("VisionTuning/pose", timestamp);
	}

	/**
	 * Gets the drive motors for the SwerveDrivetrain.
	 * @return An array of TalonFX objects representing the drive motors.
	 */
	@Override
	public TalonFX[] getDriveMotors() {
		TalonFX[] motors = new TalonFX[drivetrain.getModules().length];
		for (int i = 0; i < drivetrain.getModules().length; i++) {
			motors[i] = drivetrain.getModules()[i].getDriveMotor();
		}
		return motors;
	}

	/**
	 * Gets the steer motors for the SwerveDrivetrain.
	 * @return An array of TalonFX objects representing the steer motors.
	 */
	@Override
	public TalonFX[] getSteerMotors() {
		TalonFX[] motors = new TalonFX[drivetrain.getModules().length];
		for (int i = 0; i < drivetrain.getModules().length; i++) {
			motors[i] = drivetrain.getModules()[i].getSteerMotor();
		}
		return motors;
	}
}
