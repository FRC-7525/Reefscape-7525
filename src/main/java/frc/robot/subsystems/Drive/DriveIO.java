package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
	/**
	 * Represents the inputs for the DriveIO.
	 */
	@AutoLog
	public class DriveIOInputs {

		ChassisSpeeds speeds = new ChassisSpeeds();
		SwerveModuleState[] setPoints = new SwerveModuleState[4];
		Rotation3d fullRobotRotation = new Rotation3d();
		double odometryFrequency = 0;
		double timestamp = 0;
		double failedDataAquisitions = 0;
		double robotAngleDeg = 0;
		double gyroAngleDeg = 0;
	}

	/**
	 * Updates the inputs for the DriveIO.
	 *
	 * @param inputs The DriveIOInputs object containing the updated inputs.
	 */
	public default void updateInputs(DriveIOInputs inputs) {}

	/**
	 * Gets the SwerveDrivetrain associated with the DriveIO.
	 *
	 * @return The SwerveDrivetrain object.
	 */
	public default TunerSwerveDrivetrain getDrive() {
		return null;
	}

	/**
	 * Resets the gyro to zero.
	 */
	public default void zeroGyro() {}

	/**
	 * Sets the control for the SwerveRequest.
	 *
	 * @param request The SwerveRequest object containing the control information.
	 */
	public default void setControl(SwerveRequest request) {}

	/**
	 * Adds a vision measurement to the DriveIO.
	 *
	 * @param pose The Pose2d object representing the vision measurement.
	 * @param timestamp The timestamp of the vision measurement.
	 * @param standardDeviaton The standard deviation of the vision measurement.
	 */
	public default void addVisionMeasurement(
		Pose2d pose,
		double timestamp,
		Matrix<N3, N1> standardDeviaton
	) {}
}
