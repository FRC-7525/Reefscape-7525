package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ClimberConstants {

	public static final String SUBSYSTEM_NAME = "Climber";

	public static final int CURRENT_FILTER_TAPS = 5;

	public static final Distance POSITION_TOLERANCE = Meters.of(.1);

	public static final Distance UP = Inches.of(50);
	public static final Distance DOWN = Meters.of(0);
	public static final Distance OUTER_GEAR_DIAMETER = Millimeters.of(35.9);
	public static final double MOTOR_GEARING = 50;
	public static final Distance METERS_PER_ROTATION = Meters.of(MOTOR_GEARING * (OUTER_GEAR_DIAMETER.in(Meters) * Math.PI)); // random value lol

	public static class Sim {

		public static final int NUM_MOTORS = 1;
		public static final MomentOfInertia MOTOR_MOI = KilogramSquareMeters.of(0.001);

		public static final PIDConstants PID_CONSTANTS = new PIDConstants(2, 0, 0, 0);
	}

	public static class Real {

		public static final int CLIMBER_CANID = 18;

		public static final LinearVelocity ZEROING_VELOCITY = MetersPerSecond.of(0.25);
		public static final Current ZEROING_CURRENT_LIMIT = Amps.of(10.0);

		public static final PIDConstants PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
	}
}
