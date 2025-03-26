package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public class CoralerConstants {

	public static final String SUBSYSTEM_NAME = "Coraler";

	public static final double GEARING = 1;

	public static final Time DEBOUNCE_TIME = Seconds.of(0.075);

	public static final double SET_TO_VOLTS_CF = 12;

	public static final Supplier<PIDController> VELOCITY_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(0.05, 0, 0);
			case TESTING -> new PIDController(1, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	// States
	public static final double CORALING_VELOCITY_L4 = 0.3;
	public static final double CORALING_VELOCITY_L3_L2 = 0.4;
	public static final double CORALING_VELOCITY_L1 = 0.1;
	public static final double INTAKING_VELOCITY = 0.35;
	public static final double CENTERING_VELOCITY = 0.2;
	public static final double IDLE_VELOCITY = 0;

	public static final double STATOR_CURRENT_SENSING_LIMIT = 35;

	public static final Distance CLOSE_DISTANCE = Meter.of(0.05);

	public static class Real {

		public static final int WHEEL_MOTOR_CAN_ID = 20;
		public static final int DIO_PORT = 3;
	}

	public static class Sim {

		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.00001;
		public static final Time INTAKE_TIME = Seconds.of(0.3);
	}
}
