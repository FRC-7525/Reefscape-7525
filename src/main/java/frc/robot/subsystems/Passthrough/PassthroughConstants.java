package frc.robot.Subsystems.Passthrough;


public  class PassthroughConstants {
    public static final String SUBSYSTEM_NAME = "Powered Passthrough";

    public static final int WHEEL_MOTOR_CAN_ID = 1;
    public static final double GEARING = 1;

    public static final double INTAKING_VELOCITY = 0.2;
    public static final double OFF_VELOCITY = 0;

    public static class Sim {
		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.00001;
	}
}
