package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.Drive.Drive.SysIdMode;

public class GlobalConstants {

	public static double SIM_DELTA_TIME = 0.02;

	public enum RobotMode {
		REAL,
		TESTING,
		SIM,
		REPLAY,
	}

	public static final RobotMode ROBOT_MODE = "Crash".equals(System.getenv("CI_NAME"))
		? RobotMode.SIM
		: RobotMode.SIM;

	public static final double SIMULATION_PERIOD = 0.02;

	public static class Controllers {

		public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
		public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);
		public static final XboxController TEST_CONTROLLER = new XboxController(3);

		// NOTE: Set to 0.1 on trash controllers
		public static final double DEADBAND = 0.01;
		public static final double TRIGGERS_REGISTER_POINT = 0.5;
	}

	public static class Drive {

		public static final double SIM_UPDATE_TIME = 0.004;

		public static final AngularVelocity ANGULAR_VELOCITY_LIMIT = AngularVelocity.ofBaseUnits(
			180,
			DegreesPerSecond
		);

		// Change to change the sysID test that gets run for drive
		public static final SysIdMode SYS_ID_MODE = SysIdMode.STEER;
		public static final String SUBSYSTEM_NAME = "Drive";

		// For zeroing on robot init
		public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(
			0
		);
		public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(
			180
		);

		// Weird syntax because we have our own PIDConstants class (literally just the PP one :skull: copy pasted) so we can use it without installing PP Lib
		public static final PPHolonomicDriveController PATH_PLANNER_PID =
			new PPHolonomicDriveController(
				new com.pathplanner.lib.config.PIDConstants(5.0, 0, 0),
				new com.pathplanner.lib.config.PIDConstants(5.0, 0, 0)
			);

		public static RobotConfig geRobotConfig() {
			try {
				return RobotConfig.fromGUISettings();
			} catch (Exception e) {
				e.printStackTrace();
				// Dummy robot config, unsure of if this is good or bad
				return new RobotConfig(
					1,
					1,
					new ModuleConfig(1, 1, 1, DCMotor.getKrakenX60(1), 1, 1),
					1
				);
			}
		}

		public static final RobotConfig ROBOT_CONFIG = geRobotConfig();
	}
}
