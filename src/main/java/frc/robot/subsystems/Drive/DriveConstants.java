package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Subsystems.Drive.Drive.SysIdMode;

import static edu.wpi.first.units.Units.*;

public class DriveConstants {
    
		public static final double SIM_UPDATE_TIME = 0.004;
        
        public static final Distance WHEEL_BASE = Meters.of(0.5);

		// TODO: Change based on PP constants
		public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(5.8);

		public static final AngularVelocity ANGULAR_VELOCITY_LIMIT = AngularVelocity.ofBaseUnits(180, DegreesPerSecond);

		// Change to change the sysID test that gets run for drive
		public static final SysIdMode SYS_ID_MODE = SysIdMode.STEER;
		public static final String SUBSYSTEM_NAME = "Drive";

		// For zeroing on robot init
		public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
		public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);

		// Weird syntax because we have our own PIDConstants class (literally just the PP one :skull: copy pasted) so we can use it without installing PP Lib
		public static final PPHolonomicDriveController PATH_PLANNER_PID = new PPHolonomicDriveController(new com.pathplanner.lib.config.PIDConstants(5.0, 0, 0), new com.pathplanner.lib.config.PIDConstants(5.0, 0, 0));

		public static RobotConfig geRobotConfig() {
			try {
				return RobotConfig.fromGUISettings();
			} catch (Exception e) {
				e.printStackTrace();
				// Dummy robot config, unsure of if this is good or bad
				return new RobotConfig(1, 1, new ModuleConfig(1, 1, 1, DCMotor.getKrakenX60(1), 1, 1), 1);
			}
		}

		public static final RobotConfig ROBOT_CONFIG = geRobotConfig();
}
