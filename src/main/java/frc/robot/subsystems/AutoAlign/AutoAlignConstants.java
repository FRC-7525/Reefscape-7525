package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

	public static final Distance ROBOT_RADIUS = Meters.of(1.001);
	// public static final Distance REEF_HITBOX = Inches.of(53.7625); // Reef size = 46.75 in multiplied by 1.15 for safety
	// public static final Distance REEF_HITBOX = Inches.of(50.75); // Reef size = 46.75 in multiplied by 1.15 for safety

	//I'm pretty sure it says 37.04 inches in the specifications
	//TODO: Need to check if correct size
	//Might need to multiple hitbox to make it more consistent
	public static final Distance REEF_HITBOX = Inches.of(37.04);

	public static final Angle MIN_HEADING_ANGLE = Degrees.of(-180);
	public static final Angle MAX_HEADING_ANGLE = Degrees.of(180);

	// TODO update max speed once robot is built
	public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(15);
	public static final boolean USE_GOAL = true;
	// Sim
	public static final Distance DISTANCE_ERROR_MARGIN = Meters.of(0.0208);
	public static final Distance DISTANCE_ERROR_MARGIN2 = Meters.of(0.5);
	public static final Angle ANGLE_ERROR_MARGIN = Degrees.of(1.5); // was 0.05 radians

	// public static final Distance TRANSLATIONAL_COMPONENT_ERROR_MARGIN = Meters.of(0.15937); //2x^2 = dist error margin. solved for x
	// public static final Distance DISTANCE_ERROR_MARGIN = Meters.of(0.01); // previously 0.025
	// public static final Angle ANGLE_ERROR_MARGIN = Radians.of(0.05);

	public static final double TIMEOUT_DISTANCE_THRESHOLD = 0.05;
	public static final double TIMEOUT_THRESHOLD = 1;

	public static final double GOAL_STRENGTH = 0.1;
	static final double FIELD_LENGTH = 16.42;
	static final double FIELD_WIDTH = 8.16;

	public static final Pose2d REEF_POSE = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? new Pose2d(13.08, 4, new Rotation2d()) : new Pose2d(4.49, 4, new Rotation2d());

	public static final List<Translation2d> REEF_VERTICES = List.of(
		new Translation2d(REEF_HITBOX.in(Meters) * Math.cos(0), REEF_HITBOX.in(Meters) * Math.sin(0)).plus(REEF_POSE.getTranslation()),
		new Translation2d(REEF_HITBOX.in(Meters) * Math.cos(Math.PI / 3), REEF_HITBOX.in(Meters) * Math.sin(Math.PI / 3)).plus(REEF_POSE.getTranslation()),
		new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI * 2) / 3), REEF_HITBOX.in(Meters) * Math.sin((Math.PI * 2) / 3)).plus(REEF_POSE.getTranslation()),
		new Translation2d(REEF_HITBOX.in(Meters) * Math.cos(Math.PI), REEF_HITBOX.in(Meters) * Math.sin(Math.PI)).plus(REEF_POSE.getTranslation()),
		new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI * 4) / 3), REEF_HITBOX.in(Meters) * Math.sin((Math.PI * 4) / 3)).plus(REEF_POSE.getTranslation()),
		new Translation2d(REEF_HITBOX.in(Meters) * Math.cos((Math.PI * 5) / 3), REEF_HITBOX.in(Meters) * Math.sin((Math.PI * 5) / 3)).plus(REEF_POSE.getTranslation())
	);

	public static final List<Translation2d> REEF_EDGES = List.of(
		REEF_VERTICES.get(1).minus(REEF_VERTICES.get(0)),
		REEF_VERTICES.get(2).minus(REEF_VERTICES.get(1)),
		REEF_VERTICES.get(3).minus(REEF_VERTICES.get(2)),
		REEF_VERTICES.get(4).minus(REEF_VERTICES.get(3)),
		REEF_VERTICES.get(5).minus(REEF_VERTICES.get(4)),
		REEF_VERTICES.get(0).minus(REEF_VERTICES.get(5))
	);

	public static final List<Translation2d> ROBOT_VERTICES = List.of(
		new Translation2d(ROBOT_RADIUS.in(Meters) * Math.cos(Math.PI / 4), ROBOT_RADIUS.in(Meters) * Math.sin(Math.PI / 4)),
		new Translation2d(ROBOT_RADIUS.in(Meters) * Math.cos((Math.PI * 3) / 4), ROBOT_RADIUS.in(Meters) * Math.sin((Math.PI * 3) / 4)),
		new Translation2d(ROBOT_RADIUS.in(Meters) * Math.cos((Math.PI * 5) / 4), ROBOT_RADIUS.in(Meters) * Math.sin((Math.PI * 5) / 4)),
		new Translation2d(ROBOT_RADIUS.in(Meters) * Math.cos((Math.PI * 7) / 4), ROBOT_RADIUS.in(Meters) * Math.sin((Math.PI * 7) / 4))
	);

	public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(2);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(1);

	public static final Supplier<ProfiledPIDController> SCALED_FF_TRANSLATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new ProfiledPIDController(15, 0, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(9), 7), 0.02);
			case SIM -> new ProfiledPIDController(20, 1, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(9), 7), 0.02);
			default -> new ProfiledPIDController(20, 1, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(9), 7), 0.02);
		};

	public static final Supplier<ProfiledPIDController> SCALED_FF_ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new ProfiledPIDController(20, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2), 0.02);
			case SIM -> new ProfiledPIDController(20, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2), 0.02);
			default -> new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2), 0.02);
		};

	public static final Supplier<PIDController> REPULSOR_TRANSLATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0);
			case SIM -> new PIDController(5, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final Supplier<PIDController> REPULSOR_ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0);
			case SIM -> new PIDController(20, 0, 0);
			default -> new PIDController(10, 0, 0);
		};

	public static final class obstacles {

		public static final List<Obstacle> FIELD_OBSTACLES = List.of(
			// was 1.5 radius
			new GuidedObstacle(new Translation2d(4.48945, 4.025901), 20, true, 2.4),
			new GuidedObstacle(new Translation2d(13.10655, 4.025901), 20, true, 2.4)
		);

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
