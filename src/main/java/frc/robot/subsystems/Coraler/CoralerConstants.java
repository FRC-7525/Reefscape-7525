package frc.robot.Subsystems.Coraler;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class CoralerConstants {

	public static final String SUBSYSTEM_NAME = "Coraler";

	public static final int VELOCITY_MOTOR_CAN_ID = 1;
	public static final double GEARING = 1;

	// States
	public static final AngularVelocity CORALING_VELOCITY = RotationsPerSecond.of(2);
	public static final AngularVelocity INTAKING_VELOCITY = RotationsPerSecond.of(0);
	public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0);

	public static class Real {

		public static final PIDConstants VELOCITY_PID = new PIDConstants(0.0, 0.0, 0.0);
	}

	public static class Sim {

		public static final PIDConstants VELOCITY_PID = new PIDConstants(0.0, 0.0, 0.0);
		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.00001;
	}
}
