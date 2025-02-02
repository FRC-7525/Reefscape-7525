package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public class CoralerConstants {

	// TODO: CHANGE CONSTANTS ONCE MECH CADS THE ROBOT

	public static final String SUBSYSTEM_NAME = "Coraler";

	public static final double GEARING = 1;

	public static final Supplier<PIDController> WHEEL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.05, 0, 0);
			case SIM -> new PIDController(0.05, 0, 0);
			case TESTING -> new PIDController(1, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	// States
	public static final AngularVelocity CORALING_VELOCITY = RotationsPerSecond.of(2);
	public static final AngularVelocity INTAKING_VELOCITY = RotationsPerSecond.of(0);
	public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0);

	public static class Real {

		public static final int WHEEL_MOTOR_CAN_ID = 1;
	}

	public static class Sim {

		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.00001;
	}
}
