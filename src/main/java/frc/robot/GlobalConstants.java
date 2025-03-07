package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.Arrays;

public class GlobalConstants {

	public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
	public static final double SIMULATION_PERIOD = 0.02;
	// TODO: This is wrong
	public static final Mass ROBOT_MASS = Kilograms.of(60);

	public static final Field2d FIELD = new Field2d();

	public enum RobotMode {
		REAL,
		TESTING,
		SIM,
	}

	public static final RobotMode ROBOT_MODE = "Crash".equals(System.getenv("CI_NAME")) ? RobotMode.SIM : RobotMode.REAL;
 
	public static class Controllers {

		public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
		public static final GenericHID OPERATOR_CONTROLLER = new GenericHID(1);
		public static final XboxController TEST_CONTROLLER = new XboxController(4);

		// NOTE: Set to 0.1 on trash controllers
		public static final double DEADBAND = 0.01;
		public static final double TRIGGERS_REGISTER_POINT = 0.5;
	}

	public static class FaultManagerConstants {

		public static final ArrayList<Integer> CANIVORE_DEVICE_ORDER = new ArrayList<Integer>(Arrays.asList(39, 56, 6, 4, 58, 9, 5, 11, 12, 2, 59, 3, 8));
	}
}
