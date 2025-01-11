package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public final class AutoAlignConstants {

	public static final Distance ROBOT_RADIUS = Meters.of(0.3);
	public static final Distance REEF_HITBOX = Meters.of(1.31);

	public static final double MIN_HEADING_ANGLE = -180;
	public static final double MAX_HEADING_ANGLE = 180;

	// TODO update max speed once robot is built
	public static final double MAX_SPEED = Units.feetToMeters(15);
	public static final boolean USE_GOAL = true;
	public static final double DISTANCE_ERROR_MARGIN = .05;
	public static final double ANGLE_ERROR_MARGIN = .1;

	public static final Pose2d REEF_POSE = DriverStation.getAlliance().get() == Alliance.Red
		? new Pose2d(4.49, 4, new Rotation2d())
		: new Pose2d(13.08, 4, new Rotation2d());

	public static final Supplier<PIDController> TRANSLATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(1, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final Supplier<PIDController> ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(1, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final Supplier<PIDController> REPULSOR_TRANSLATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(1, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final Supplier<PIDController> REPULSOR_ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(1, 0, 0);
			default -> new PIDController(1, 0, 0);
		};
}
