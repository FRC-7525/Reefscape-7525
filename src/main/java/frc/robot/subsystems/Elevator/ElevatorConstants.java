package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import org.team7525.controlConstants.FFConstants;

public final class ElevatorConstants {

	public static final String SUBSYSTEM_NAME = "Elevator";

	public static final int LEFT_MOTOR_CANID = 12;
	public static final int RIGHT_MOTOR_CANID = 13;

	public static final Distance POSITION_TOLERANCE = Meters.of(.1);
	public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(.1);
	public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS =
		new TrapezoidProfile.Constraints(1, .5);
	public static final LinearVelocity ZEROING_VELOCITY = MetersPerSecond.of(0.25);
	public static final Current ZEROING_CURRENT_LIMIT = Amps.of(10.0);

	public static final Distance HIGH_POSITION_HEIGHT = Meters.of(2.5146);
	public static final Distance MID_POSITION_HEIGHT = Meters.of(1.54305); //mid is all the way down according to nick
	public static final Distance DOWN_POSITION_HEIGHT = Meters.of(1.54305);
	public static final Distance METERS_PER_ROTATION = Meters.of(1); // random value lol

	public static class Sim {

		// TODO change all of these values when new robot is done
		public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
		public static final double GEARING = 9;
		public static final Mass CARRIAGE_MASS = Kilogram.of(28.44);
		public static final Distance DRUM_RADIUS = Meters.of(.5); // Random value cuz mech is bum
		public static final Distance MIN_HEIGHT = Meters.of(1.54305);
		public static final Distance MAX_HEIGHT = Meters.of(2.60985);
		public static final boolean SIMULATE_GRAVITY = true;
		public static final Distance STARTING_HEIGHT = Meters.of(1.54305);

		// TODO Tune once we get new values
		public static final PIDConstants PROFILLED_PID_CONSTANTS = new PIDConstants(2, 0, 0, 0);

		public static final FFConstants FF_CONSTANTS = new FFConstants(.2, .2, 0, 0);
	}

	public static class Real {

		// TODO get rid of limit switch cuz otto said so
		public static final int LIMIT_SWITCH_DIO = 1;

		public static final boolean LEFT_INVERTED = false;
		public static final NeutralModeValue LEFT_NEUTRAL_MODE = NeutralModeValue.Brake;
		public static final boolean LEFT_STRATOR_CURRENT_LIMIT_ENABLED = true;
		public static final Current LEFT_STRATOR_CURRENT_LIMIT = Amps.of(30);

		public static final boolean RIGHT_INVERTED = false;
		public static final NeutralModeValue RIGHT_NEUTRAL_MODE = NeutralModeValue.Brake;
		public static final boolean RIGHT_STRATOR_CURRENT_LIMIT_ENABLED = true;
		public static final Current RIGHT_STRATOR_CURRENT_LIMIT = Amps.of(30);

		public static final PIDConstants PROFILLED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
		public static final FFConstants FF_CONSTANTS = new FFConstants(0, 0, 0, 0);
	}
}
