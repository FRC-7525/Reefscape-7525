package frc.robot.Subsystems.Algaer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public final class AlgaerConstants {

	public static final String SUBSYSTEM_NAME = "Algaer";

	public static final double INTAKING_SPEED = -0.3;
	public static final double PROCESSING_SPEED = 0.5;
	public static final double HOLDING_SPEED = -0.05;
	public static final double IDLE_SPEED = 0;

	// TODO: FIND!
	public static final double OVERALL_GEARING = 50;

	public static final Angle INTAKING_PIVOT = Degrees.of(-90);
	public static final Angle PROCESSING_PIVOT = Degrees.of(-90);
	public static final Angle HOLDING_PIVOT = Degrees.of(-30);
	public static final Angle IDLE_PIVOT = Degrees.of(-10);

	public static final Angle PIVOT_TOLERANCE = Degrees.of(15);
	public static final AngularVelocity WHEEL_TOLERANCE = RotationsPerSecond.of(5);

	public static final Supplier<PIDController> PIVOT_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.1, 0, 0.01);
			case SIM -> new PIDController(0.05, 0, 0.01);
			case TESTING -> new PIDController(0.1, 0, 0);
			default -> new PIDController(0, 0, 0);
		};

	public static final class Sim {

		public static final int NUM_PIVOT_MOTORS = 1;
		public static final MomentOfInertia PIVOT_MOI = KilogramSquareMeters.of(0.31654227);
		public static final Distance PIVOT_ARM_LENGTH = Meters.of(.26199558);
		public static final Angle MIN_PIVOT_ANGLE = Degrees.of(0);
		public static final Angle MAX_PIVOT_ANGLE = Degrees.of(130);
		public static final Angle STARTING_PIVOT_ANGLE = Degrees.of(0);

		public static final int NUM_WHEEL_MOTORS = 1;
		public static final MomentOfInertia WHEEL_MOTOR_MOI = KilogramSquareMeters.of(0.0001);
		public static final double WHEEL_MOTOR_GEARING = 3;

		public static final Translation3d ZEROED_TRANSLATION = new Translation3d(0.3, 0, 0.355);
	}

	public static final class Real {

		public static final int WHEEL_MOTOR_CANID = 29;
		public static final int PIVOT_MOTOR_CANID = 15;
	}
}
