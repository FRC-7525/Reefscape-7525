package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Sim.DRUM_RADIUS;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Sim.GEARING;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import org.team7525.controlConstants.FFConstants;

public final class ElevatorConstants {

	public static final String SUBSYSTEM_NAME = "Elevator";

	public static final int LEFT_MOTOR_CANID = 12;
	public static final int RIGHT_MOTOR_CANID = 13;

	public static final Distance POSITION_TOLERANCE = Inches.of(0.5);
	public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(0.1);
	// TODO: Set to smaller numbers once we have robot (low max vel/acc for testing, real should be around 100)
	public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(148.64);
	public static final LinearAcceleration MAX_ACCELERATION = InchesPerSecond.per(Second).of(150);

	public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY.in(MetersPerSecond), MAX_ACCELERATION.in(MetersPerSecondPerSecond));
	public static final LinearVelocity ZEROING_VELOCITY = InchesPerSecond.of(-4);
	public static final Current ZEROING_CURRENT_LIMIT = Amps.of(10.0);

	// TODO: All of these are RANDOM values, once the robot is built GET REAL ONES
	public static final Distance L4_HEIGHT = Inches.of(24);
	public static final Distance L3_HEIGHT = Inches.of(12); //mid is all the way down according to nick
	public static final Distance L2_HEIGHT = Inches.of(5);
	public static final Distance L1_HEIGHT = Inches.of(0);
	public static final Distance IDLE_HEIGHT = Meters.of(0);
	public static final Distance ALGAE_LOW_HEIGHT = Meters.of(1);
	public static final Distance ALGAE_HIGH_HEIGHT = Meters.of(1);
	public static final Distance ALGAE_PROCESSOR_HEIGHT = Inches.of(0);

	public static final Distance METERS_PER_ROTATION = Meters.of(GEARING * (2 * Math.PI * DRUM_RADIUS.in(Meters))); // double check if this is right

	public static class Sim {

		public static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(2);
		public static final double GEARING = 2;
		public static final Mass CARRIAGE_MASS = Pounds.of(15);
		public static final Distance DRUM_RADIUS = Inches.of(0.5);
		public static final Distance MIN_HEIGHT = Inches.of(0);
		public static final Distance MAX_HEIGHT = Inches.of(24);
		public static final boolean SIMULATE_GRAVITY = true;
		public static final Distance STARTING_HEIGHT = Inches.of(0);

		public static final PIDConstants PROFILLED_PID_CONSTANTS = new PIDConstants(30, 0, 0, 0);
		public static final FFConstants FF_CONSTANTS = new FFConstants(0.74, 0.27, 3.11, 0.03);
	}

	public static class Real {

		public static final boolean LEFT_INVERTED = false;
		public static final NeutralModeValue LEFT_NEUTRAL_MODE = NeutralModeValue.Brake;
		public static final boolean LEFT_STRATOR_CURRENT_LIMIT_ENABLED = true;
		public static final Current LEFT_STRATOR_CURRENT_LIMIT = Amps.of(40);

		public static final boolean RIGHT_INVERTED = false;
		public static final NeutralModeValue RIGHT_NEUTRAL_MODE = NeutralModeValue.Brake;
		public static final boolean RIGHT_STRATOR_CURRENT_LIMIT_ENABLED = true;
		public static final Current RIGHT_STRATOR_CURRENT_LIMIT = Amps.of(40);

		public static final PIDConstants PROFILLED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
		// The move is prob to keep this at 0 and not bother tuning
		public static final FFConstants FF_CONSTANTS = new FFConstants(0, 0, 0, 0);
	}
}
