package frc.robot.Subsystems.AprilTagVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {

	public static enum CameraResolution {
		HIGH_RES,
		NORMAL,
	}

	public static final Matrix<N3, N1> highResSingleTagStdDev = VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);
	public static final Matrix<N3, N1> normalSingleTagStdDev = VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);
	public static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 3);
	public static final Matrix<N3, N1> normalMultiTagStdDev = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);

	// Robot to card
	public static final String BACK_CAMERA_NAME = "Back Left Camera";
	public static final String FRONT_CAMERA_NAME = "Front Right Camera";

	public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

	public static final Translation3d ROBOT_TO_BACK_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-11.697), Units.inchesToMeters(11.81), Units.inchesToMeters(8.859));
	public static final Rotation3d ROBOT_TO_BACK_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180));
	public static final Transform3d ROBOT_TO_BACK_CAMERA = new Transform3d(ROBOT_TO_BACK_CAMERA_TRALSLATION, ROBOT_TO_BACK_CAMERA_ROTATION);
	public static final Translation3d ROBOT_TO_FRONT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(11.697), Units.inchesToMeters(-11.81), Units.inchesToMeters(8.859));
	public static final Rotation3d ROBOT_TO_FRONT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-15), Math.toRadians(0));
	public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(ROBOT_TO_FRONT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_CAMERA_ROTATION);

	public static final double CAMERA_DEBOUNCE_TIME = 0.5;

	// Other
	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

	public static final int CAMERA_WIDTH = 1200;
	public static final int CAMERA_HEIGHT = 800;
	public static final Rotation2d CAMERA_ROTATION = Rotation2d.fromDegrees(84.47);
	public static final double CALIB_ERROR_AVG = 0.25;
	public static final double CALIB_ERROR_STD_DEV = 0.08;
	public static final int CAMERA_FPS = 40;
	public static final int AVG_LATENCY_MS = 40;
	public static final int LATENCY_STD_DEV_MS = 10;
}
