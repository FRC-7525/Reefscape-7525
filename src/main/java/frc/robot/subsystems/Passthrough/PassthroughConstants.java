package frc.robot.Subsystems.Passthrough;

public class PassthroughConstants {

	public static final String SUBSYSTEM_NAME = "Powered Passthrough";

	public static final int WHEEL_MOTOR_CAN_ID = 29;
	public static final double GEARING = 1;

	public static final double INTAKING_VELOCITY = -0.7;
	public static final double OUTTAKING_VELOCITY = 0.05;
	public static final double OFF_VELOCITY = 0;

	public static class Sim {

		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.00001;
		public static final double SET_TO_VOLTS_CF = 12;
	}
}
