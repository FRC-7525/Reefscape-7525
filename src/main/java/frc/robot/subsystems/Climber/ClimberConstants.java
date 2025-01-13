package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ClimberConstants {

	public static final String SUBSYSTEM_NAME = "Climber";

	public static final int MOTOR_CANID = 12; //TODO: Change CAN ID to actually correct one

	public static final int CURRENT_FILTER_TAPS = 5;

	public static final Distance POSITION_TOLERANCE = Meters.of(.1);

	public static final Distance UP = Meters.of(2.5146);
	public static final Distance IDLE = Meters.of(0); //TODO: Change these setpoint distances to something that is actually real
	public static final Distance DOWN = Meters.of(1.54305);
	public static final Distance METERS_PER_ROTATION = Meters.of(1); // random value lol

	public static class Sim {

		// TODO change all of these values when new robot is done
		public static final int NUM_MOTORS = 1;
		public static final MomentOfInertia MOTOR_MOI = KilogramSquareMeters.of(1); //TODO: Get an actual value for htis;
		public static final double MOTOR_GEARING = 9;

		// TODO Tune once we get new values
		public static final PIDConstants PID_CONSTANTS = new PIDConstants(2, 0, 0, 0);
	}

	public static class Real {

		public static final LinearVelocity ZEROING_VELOCITY = MetersPerSecond.of(0.25);
		public static final Current ZEROING_CURRENT_LIMIT = Amps.of(10.0);

		public static final PIDConstants PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
	}
}
