package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.GlobalConstants;
import java.util.List;
import java.util.function.Supplier;
import org.team7525.autoAlign.RepulsorFieldPlanner.GuidedObstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.HorizontalObstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.Obstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.VerticalObstacle;

public final class AutoAlignConstants {

	public static final Distance ROBOT_RADIUS = Meters.of(1.1);
	public static final Distance REEF_HITBOX = Meters.of(0.55);

	public static final Angle MIN_HEADING_ANGLE = Degrees.of(-180);
	public static final Angle MAX_HEADING_ANGLE = Degrees.of(180);

	// TODO update max speed once robot is built
	public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(15);
	public static final boolean USE_GOAL = true;
	// Sim
	// public static final double DISTANCE_ERROR_MARGIN = 0.1;
	// public static final double ANGLE_ERROR_MARGIN = 0.1;

	public static final double DISTANCE_ERROR_MARGIN = 0.025; // Real values
	public static final double ANGLE_ERROR_MARGIN = 0.1;

	public static final double GOAL_STRENGTH = 0.65;
	static final double FIELD_LENGTH = 16.42;
	static final double FIELD_WIDTH = 8.16;

	public static final Pose2d REEF_POSE = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? new Pose2d(13.08, 4, new Rotation2d()) : new Pose2d(4.49, 4, new Rotation2d());

	// TODO tune these
	public static final Supplier<PIDController> TRANSLATIONALY_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(7, 0, 0);
			case SIM -> new PIDController(5, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final Supplier<PIDController> TRANSLATIONALX_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(7, 0, 0);
			case SIM -> new PIDController(5, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(2);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(1);

	public static final Supplier<ProfiledPIDController> ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new ProfiledPIDController(10, 0, 0, new Constraints(MAX_ANGULAR_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
			case SIM -> new ProfiledPIDController(10, 0, 0.2, new Constraints(MAX_ANGULAR_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
			default -> new ProfiledPIDController(0.1, 0, 0.2, new Constraints(MAX_ANGULAR_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
		};

	public static final Supplier<PIDController> REPULSOR_TRANSLATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0);
			case SIM -> new PIDController(1, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final Supplier<PIDController> REPULSOR_ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0.1);
			case SIM -> new PIDController(20, 0, 0);
			default -> new PIDController(10, 0, 0);
		};

	public static final class obstacles {

		public static final List<Obstacle> FIELD_OBSTACLES = List.of(new GuidedObstacle(new Translation2d(4.49, 4), 6, true, 1.2), new GuidedObstacle(new Translation2d(13.08, 4), 6, true, 1.2));

		public static final List<Obstacle> WALLS = List.of(
			new HorizontalObstacle(0.0, 0.5, true),
			new HorizontalObstacle(FIELD_WIDTH, 0.5, false),
			new VerticalObstacle(0.0, 0.5, true),
			new VerticalObstacle(FIELD_LENGTH, 0.5, false),
			new VerticalObstacle(7.55, 0.5, false),
			new VerticalObstacle(10, 0.5, true)
		);
	}
}
