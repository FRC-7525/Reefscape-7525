package frc.robot.Autonomous;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
	private final AutoCommands autoCommands = new AutoCommands();

	private AutoManager() {
		// Logging Config
		PathPlannerLogging.setLogActivePathCallback(poses -> {
			tippingCalculator.willTip(null, null);
		});
		PathPlannerLogging.setLogActivePathCallback(activePath -> {
			Logger.recordOutput("Auto/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));

			if (GlobalConstants.ROBOT_MODE == GlobalConstants.RobotMode.SIM) {
				// NOTE: ASSUMES THAT THE LAST POSE IN THE CURRENT PATH HAS A TARGET VELOCITY OF 0 m/s, WILL ONLY REALLY WORK FOR THE END OF THE PATH!!!!
				Logger.recordOutput("Auto/Tipping", tippingCalculator.willTip(activePath.get(activePath.size() - 1), Drive.getInstance().getPose(), Drive.getInstance().getVelocity()));
			}
		});
		PathPlannerLogging.setLogTargetPoseCallback(targetPose -> {
			Logger.recordOutput("Auto/TrajectorySetpoint", targetPose);
		});

		// Name Commands
		NamedCommands.registerCommand("Score L4", autoCommands.new ScoreReef(4));
		NamedCommands.registerCommand("Score L3", autoCommands.new ScoreReef(3));
		NamedCommands.registerCommand("Score L2", autoCommands.new ScoreReef(2));
		NamedCommands.registerCommand("Score L1", autoCommands.new ScoreReef(1));
		NamedCommands.registerCommand("Intake Coral L", autoCommands.new IntakeCoral(true));
		NamedCommands.registerCommand("Intake Coral R", autoCommands.new IntakeCoral(false));
		NamedCommands.registerCommand("Process Algae", autoCommands.new ProcessAlgae());

		// Autos
		autoChooser.setDefaultOption("Do Nothing", new PrintCommand("Do Nothing"));
		autoChooser.addOption("4 Note", new PathPlannerAuto("4 Note Testing"));
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
