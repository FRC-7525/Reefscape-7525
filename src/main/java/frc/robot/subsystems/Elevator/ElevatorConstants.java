package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Sim.DRUM_RADIUS;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Sim.GEARING;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import org.team7525.coefficients.PIDCoefficients;
import org.team7525.controlConstants.FFConstants;

public final class ElevatorConstants {

	public static final String SUBSYSTEM_NAME = "Elevator";

	public static final int LEFT_MOTOR_CANID = 12;
	public static final int RIGHT_MOTOR_CANID = 13;

	public static final Distance POSITION_TOLERANCE = Meter.of(0.1);
	public static final Distance NEAR_ENOUGH_POSITION = Inches.of(10);
	public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(0.1);
	// TODO: Set to smaller numbers once we have robot (low max vel/acc for testing, real should be around 100)
	// public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(10);
	// public static final LinearAcceleration MAX_ACCELERATION = InchesPerSecond.per(Second).of(10);
	public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(60);
	public static final LinearAcceleration MAX_ACCELERATION = InchesPerSecond.per(Second).of(150);

	public static final double TRIGGER_THRESHOLD = 0.1;

	public static final boolean TOGGLE_MANUAL_CONTROL = true;
	//TODO: Tune for use in real
	//It seems that if its too small, then the change is within the PID tolerance
	//and the elevator doesn't bother moving and so it no longer works
	public static final Distance MANUAL_HEIGHT_CHANGE = Inches.of(0.25);

	public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY.in(MetersPerSecond), MAX_ACCELERATION.in(MetersPerSecondPerSecond));
	//TODO: Tune in real life so that its not so fast that it slams but also fast enough
	public static final double ZEROING_SPEED = -0.15;

	public static final Distance TRANSITION_HEIGHT = Inches.of(12);
	public static final Distance L4_HEIGHT = Inches.of(23.2);
	public static final Distance L3_HEIGHT = Inches.of(15.5);
	public static final Distance L2_HEIGHT = Inches.of(10);
	public static final Distance L1_HEIGHT = Inches.of(7.5); // was 5
	public static final Distance L1_SCORING_HEIGHT = Inches.of(10); //TODO: TUNE
	public static final Distance IDLE_HEIGHT = Inches.of(0);

	public static final Distance METERS_PER_ROTATION = Meters.of((1 / GEARING) * (2 * Math.PI * DRUM_RADIUS.in(Meters))); // double check if this is right

	public static class Sim {

		public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
		public static final double GEARING = 9;
		public static final Mass CARRIAGE_MASS = Pounds.of(34.544);
		public static final Distance DRUM_RADIUS = Inches.of(1.751).div(2);
		public static final Distance MIN_HEIGHT = Inches.of(0);
		public static final Distance MAX_HEIGHT = Inches.of(24);
		public static final boolean SIMULATE_GRAVITY = true;
		public static final Distance STARTING_HEIGHT = Inches.of(0);

		public static final PIDCoefficients PROFILLED_PID_CONSTANTS = new PIDCoefficients(200, 0, 0, 0.2);
		public static final FFConstants FF_CONSTANTS = new FFConstants(0, 0.11, 7, 0.01);

		// Sim is trolling, idk why
		public static final Distance POSITION_TOLERANCE_SIM = Inches.of(4);
		public static final LinearVelocity VELOCITY_TOLERANCE_SIM = InchesPerSecond.of(0.5);
	}

	public static class Real {

		public static final boolean LEFT_INVERTED = false;
		public static final NeutralModeValue LEFT_NEUTRAL_MODE = NeutralModeValue.Brake;
		public static final boolean LEFT_STRATOR_CURRENT_LIMIT_ENABLED = true;
		public static final Current LEFT_STRATOR_CURRENT_LIMIT = Amps.of(100);

		public static final boolean RIGHT_INVERTED = false;
		public static final NeutralModeValue RIGHT_NEUTRAL_MODE = NeutralModeValue.Brake;
		public static final boolean RIGHT_STRATOR_CURRENT_LIMIT_ENABLED = true;
		public static final Current RIGHT_STRATOR_CURRENT_LIMIT = Amps.of(100);

		public static final PIDCoefficients PROFILLED_PID_CONSTANTS = new PIDCoefficients(30, 0, 1, 0);
		// The move is prob to keep this at 0 and not bother tuning
		// public static final FFConstants FF_CONSTANTS = new FFConstants(0, 0.61, 3.11, 0.06);
		public static final FFConstants FF_CONSTANTS = new FFConstants(0, 0.11, 4, 0.01);
	}
}
