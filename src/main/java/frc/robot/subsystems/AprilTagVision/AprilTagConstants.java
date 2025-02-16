package frc.robot.Subsystems.AprilTagVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public final class AprilTagConstants {

	public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

	public static final String SUBSYSTEM_NAME = "AprilTagVision";
	
	public static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

	public static enum CameraResolution {
		HIGH_RES,
		NORMAL,
	}

	// Front Left 
	public static final String FRONT_LEFT_CAM_NAME = "Front Left Camera";
	public static final Translation3d ROBOT_TO_FRONT_LEFT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(11.697), Units.inchesToMeters(11.81), Units.inchesToMeters(8.859));
	public static final Rotation3d ROBOT_TO_FRONT_LEFT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-15), Math.toRadians(35));
	public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(ROBOT_TO_FRONT_LEFT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_LEFT_CAMERA_ROTATION);


	// Front Right
	public static final String FRONT_RIGHT_CAM_NAME = "Front Right Camera";
	public static final Translation3d ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(11.697), Units.inchesToMeters(-11.81), Units.inchesToMeters(8.859));
	public static final Rotation3d ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-15), Math.toRadians(0));
	public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION);
	
	//Back Left
	public static final String BACK_LEFT_CAM_NAME = "Back Left Camera";
	public static final Translation3d ROBOT_TO_BACK_LEFT_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-11.697), Units.inchesToMeters(11.81), Units.inchesToMeters(8.859));
	public static final Rotation3d ROBOT_TO_BACK_LEFT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180));
	public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(ROBOT_TO_BACK_LEFT_CAMERA_TRALSLATION, ROBOT_TO_BACK_LEFT_CAMERA_ROTATION);

	//Back Right
		//Back Left
		public static final String BACK_RIGHT_CAM_NAME = "Back Right Camera";
		public static final Translation3d ROBOT_TO_BACK_RIGHT_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-11.697), Units.inchesToMeters(-11.81), Units.inchesToMeters(8.859));
		public static final Rotation3d ROBOT_TO_BACK_RIGHT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-15), Math.toRadians(215));
		public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(ROBOT_TO_BACK_RIGHT_CAMERA_TRALSLATION, ROBOT_TO_BACK_RIGHT_CAMERA_ROTATION);
	

	public static final Matrix<N3, N1> highResSingleTagStdDev = VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);
	public static final Matrix<N3, N1> normalSingleTagStdDev = VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);
	public static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 3);
	public static final Matrix<N3, N1> normalMultiTagStdDev = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
}
