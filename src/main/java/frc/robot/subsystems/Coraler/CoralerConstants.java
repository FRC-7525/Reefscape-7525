package frc.robot.subsystems.Coraler;

import com.pathplanner.lib.config.PIDConstants;

public class CoralerConstants {

	public static final String SUBSYSTEM_NAME = "Coraler";

	public static final int VELOCITY_MOTOR_CAN_ID = 1;
	public static final double GEARING = 1;

	public static class Real {

		public static final PIDConstants VELOCITY_PID = new PIDConstants(0.0, 0.0, 0.0);
	}

	public static class Sim {

		public static final PIDConstants VELOCITY_PID = new PIDConstants(0.0, 0.0, 0.0);
		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.00001;
	}
}
