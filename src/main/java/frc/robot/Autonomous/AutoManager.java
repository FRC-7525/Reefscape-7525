package frc.robot.Autonomous;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class AutoManager {

	public static AutoManager instance;

	private final SendableChooser<Command> autoChooser = new SendableChooser<>();
	private final TippingCalculator tippingCalculator = new TippingCalculator(GlobalConstants.ROBOT_MASS, DriveConstants.WHEEL_BASE);

	private AutoManager() {
		// Logging Config
		PathPlannerLogging.setLogActivePathCallback(poses -> {
			tippingCalculator.willTip(null, null);
		});
		PathPlannerLogging.setLogActivePathCallback(activePath -> {
			Logger.recordOutput("Auto/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));

			if (GlobalConstants.ROBOT_MODE == GlobalConstants.RobotMode.SIM) {
				// NOTE: ASSUMES THAT THE LAST POSE IN THE CURRENT PATH HAS A TARGET VELOCITY OF 0 m/s
				Logger.recordOutput("Auto/Tipping", tippingCalculator.willTip(activePath.get(activePath.size() - 1), Drive.getInstance().getPose(), Drive.getInstance().getVelocity()));
			}
		});
		PathPlannerLogging.setLogTargetPoseCallback(targetPose -> {
			Logger.recordOutput("Auto/TrajectorySetpoint", targetPose);
		});
	}

	public static AutoManager getInstance() {
		if (instance == null) {
			instance = new AutoManager();
		}
		return instance;
	}

	public Command getSelectedCommand() {
		if (autoChooser.getSelected() == null) {
			return new PrintCommand("No auto selected");
		}

		return autoChooser.getSelected();
	}
}
