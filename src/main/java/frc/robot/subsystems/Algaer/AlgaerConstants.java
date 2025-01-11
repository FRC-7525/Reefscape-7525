package frc.robot.subsystems.Algaer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class AlgaerConstants {

	public static final AngularVelocity INTAKING_SPEED = RotationsPerSecond.of(60);
	public static final AngularVelocity OUTTAKING_SPEED = RotationsPerSecond.of(-60);
	public static final AngularVelocity PASSING_SPEED = RotationsPerSecond.of(30); //might be negative idk
	public static final AngularVelocity IDLE_SPEED = RotationsPerSecond.of(0);

	public static final Angle INTAKING_PIVOT = Degrees.of(70);
	public static final Angle OUTTAKING_PIVOT = Degrees.of(70);
	public static final Angle PASSING_PIVOT = Degrees.of(0);
	public static final Angle IDLE_PIVOT = Degrees.of(0);

	public static final class Sim {

		public static final int NUM_PIVOT_MOTORS = 1;
		public static final double PIVOT_GEARING = 5;
		public static final MomentOfInertia PIVOT_MOI = KilogramSquareMeters.of(0.31654227);
		public static final Distance PIVOT_ARM_LENGTH = Meters.of(.26199558);
		public static final Angle MIN_PIVOT_ANGLE = Degrees.of(0);
		public static final Angle MAX_PIVOT_ANGLE = Degrees.of(70);
		public static final Angle STARTING_PIVOT_ANGLE = Degrees.of(0);
		public static final PIDConstants PIVOT_PID_CONSTANTS = new PIDConstants(3, 0, 0);

		public static final int NUM_WHEEL_MOTORS = 1;
		public static final MomentOfInertia WHEEL_MOTOR_MOI = KilogramSquareMeters.of(1); // random value
		public static final double WHEEL_MOTOR_GEARING = 3;
		public static final PIDConstants WHEEL_PID_CONSTANTS = new PIDConstants(0.0012, 0, 0);
	}

	public static final class Real {

		public static final int WHEEL_MOTOR_CANID = 10; //TODO: Get actual CAN IDs
		public static final int PIVOT_MOTOR_CANID = 11;
		public static final PIDConstants PIVOT_PID_CONSTANTS = new PIDConstants(3, 0, 0);
		public static final PIDConstants WHEEL_PID_CONSTANTS = new PIDConstants(0.0012, 0, 0);
	}
}
