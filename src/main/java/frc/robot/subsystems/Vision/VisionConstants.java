package frc.robot.Subsystems.Vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import org.team7525.misc.VisionUtil.CameraResolution;

public class VisionConstants {

	// Robot to cam
	public static final Translation3d ROBOT_TO_BACK_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(11.5), Units.inchesToMeters(9));
	public static final Rotation3d ROBOT_TO_BACK_CAMERA_ROTATION = new Rotation3d(
		0,
		Math.toRadians(-10), // pitch i think
		Math.toRadians(-10) // yaw i think
	);
    public static final String BACK_CAMERA_NAME = "Back Camera";
    public static final String FRONT_CAMERA_NAME = "Front Camera";

	public static final Transform3d ROBOT_TO_BACK_CAMERA = new Transform3d(ROBOT_TO_BACK_CAMERA_TRALSLATION, ROBOT_TO_BACK_CAMERA_ROTATION);
	public static final Translation3d ROBOT_TO_FRONT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(11.5), Units.inchesToMeters(-11.5), Units.inchesToMeters(9));
	public static final Rotation3d ROBOT_TO_FRONT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(-15));
	public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(ROBOT_TO_FRONT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_CAMERA_ROTATION);

	public static final double CAMERA_DEBOUNCE_TIME = 0.5;

	// TODO: What camera resolutions actually are these? Assuming they're high bc
	// 1080p is high
	public static final CameraResolution BACK_RESOLUTION = CameraResolution.HIGH_RES;
	public static final CameraResolution FRONT_RESOLUTION = CameraResolution.HIGH_RES;

	// Other
	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

	static {
		AprilTagFieldLayout layout = null;
		try {
			layout = new AprilTagFieldLayout("src\\main\\java\\frc\\robot\\Subsystems\\Vision\\2025-reefscape.json");
		} catch (IOException e) {
			e.printStackTrace();
		}
		APRIL_TAG_FIELD_LAYOUT = layout;
	}

	public static final int CAMERA_WIDTH = 1200;
	public static final int CAMERA_HEIGHT = 800;
	public static final Rotation2d CAMERA_ROTATION = Rotation2d.fromDegrees(84.47);
	public static final double CALIB_ERROR_AVG = 0.25;
	public static final double CALIB_ERROR_STD_DEV = 0.08;
	public static final int CAMERA_FPS = 40;
	public static final int AVG_LATENCY_MS = 40;
	public static final int LATENCY_STD_DEV_MS = 10;
}