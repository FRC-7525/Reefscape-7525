package frc.robot.Subsystems.Vision;

import org.team7525.misc.VisionUtil.CameraResolution;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    // Robot to cam
    public static final Translation3d ROBOT_TO_BACK_CAMERA_TRALSLATION = new Translation3d(
            Units.inchesToMeters(-14.25),
            Units.inchesToMeters(-14.25),
            Units.inchesToMeters(5));
    public static final Rotation3d ROBOT_TO_BACK_CAMERA_ROTATION = new Rotation3d(
            0,
            Math.toRadians(-15), // pitch i think
            Math.toRadians(-160) // yaw i think
    );
    public static final Transform3d ROBOT_TO_BACK_CAMERA = new Transform3d(
            ROBOT_TO_BACK_CAMERA_TRALSLATION,
            ROBOT_TO_BACK_CAMERA_ROTATION);
    public static final Translation3d ROBOT_TO_FRONT_CAMERA_TRANSLATION = new Translation3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(5));
    public static final Rotation3d ROBOT_TO_FRONT_CAMERA_ROTATION = new Rotation3d(
            0,
            Math.toRadians(-10),
            Math.toRadians(190));
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
            ROBOT_TO_FRONT_CAMERA_TRANSLATION,
            ROBOT_TO_FRONT_CAMERA_ROTATION);

    public static final double CAMERA_DEBOUNCE_TIME = 0.5;

    // TODO: What camera resolutions actually are these? Assuming they're high bc
    // 1080p is high
    public static final CameraResolution BACK_RESOLUTION = CameraResolution.HIGH_RES;
    public static final CameraResolution FRONT_RESOLUTION = CameraResolution.HIGH_RES;

    // Other
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2024Crescendo);

}
