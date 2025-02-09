package frc.robot.Autonomous;

import static frc.robot.Autonomous.AutoConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class AutoManager {

	public static AutoManager instance;

	private final SendableChooser<Command> autoChooser = new SendableChooser<>();
	private final TippingCalculator tippingCalculator = new TippingCalculator(GlobalConstants.ROBOT_MASS, DriveConstants.WHEEL_BASE);

	private LinearVelocity cachedVelocity;
	private int loopCount;

	private AutoManager() {
		this.cachedVelocity = Drive.getInstance().getVelocity();
		this.loopCount = 0;

		// Logging Config (path planner)
		PathPlannerLogging.setLogActivePathCallback(poses -> {
			Logger.recordOutput("Auto/poses", poses.toArray(new Pose2d[poses.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback(targetPose -> {
			Logger.recordOutput("Auto/TrajectorySetpoint", targetPose);
		});
		PathPlannerLogging.setLogActivePathCallback(activePath -> {
			Logger.recordOutput("Auto/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));

			// Logging for tipping
			if (GlobalConstants.ROBOT_MODE == GlobalConstants.RobotMode.SIM) {
				Distance cgHeight = calculateVerticalCG(Elevator.getInstance().getHeight());
				tippingCalculator.updateCGHeight(cgHeight);

				if (activePath.size() > 0) {
					Logger.recordOutput("Auto/Tipping", tippingCalculator.willTip(activePath.get(activePath.size() - 1), Drive.getInstance().getPose(), Drive.getInstance().getVelocity()));
				}
				// Half a second, assumes non significant loop overuns (which is fine)
				loopCount += 1;
				if (loopCount % 25 == 0) {
					Logger.recordOutput("Auto/Velocity Calculated Tipping", tippingCalculator.willTip(Drive.getInstance().getVelocity(), cachedVelocity, TIPPING_CALCULATION_TIME));
					cachedVelocity = Drive.getInstance().getVelocity();
					loopCount = 0;
				}
			}
		});

		// Name Commands
		NamedCommands.registerCommand("Score L4", AutoCommands.ScoreReef.atLevel(4));
		NamedCommands.registerCommand("Score L3", AutoCommands.ScoreReef.atLevel(3));
		NamedCommands.registerCommand("Score L2", AutoCommands.ScoreReef.atLevel(2));
		NamedCommands.registerCommand("Score L1", AutoCommands.ScoreReef.atLevel(1));

		NamedCommands.registerCommand("Transition L4", AutoCommands.GoToElevatorLevel.atLevel(4));
		NamedCommands.registerCommand("Transition L3", AutoCommands.GoToElevatorLevel.atLevel(3));
		NamedCommands.registerCommand("Transition L2", AutoCommands.GoToElevatorLevel.atLevel(2));
		NamedCommands.registerCommand("Transition L1", AutoCommands.GoToElevatorLevel.atLevel(1));

		NamedCommands.registerCommand("Return To Idle", AutoCommands.ReturnToIdle.get());

		NamedCommands.registerCommand("Intake Coral", AutoCommands.IntakeCoral.getCoral());

		// Autos

		// Misc
		autoChooser.setDefaultOption("Do Nothing", new PrintCommand("Do Nothing"));
		autoChooser.addOption("Cross Line", new PathPlannerAuto("Cross Line"));
		autoChooser.addOption("PID Tuning", new PathPlannerAuto("PID Tuning"));

		// 1 Coral
		autoChooser.addOption("C1 | BN1 | G", new PathPlannerAuto("C1, BN1, G"));
		autoChooser.addOption("C1 | UN1 | H", new PathPlannerAuto("C1, UN1, H"));

		// 2 Coral

		// 3 Coral

		// 4 Coral
		autoChooser.addOption("C4 | UN1 | I, J, K, L", new PathPlannerAuto("C4, UN1, {I, J, K, L}"));
		autoChooser.addOption("C4 | BN1 | C, D, E, F", new PathPlannerAuto("C4, BN1, { C, D, E, F}"));

		// 5 Coral
		autoChooser.addOption("C5 | UN1 | A, I, J, K, L", new PathPlannerAuto("C5, UN1, {A, I, J, K, L}"));
		autoChooser.addOption("C5 | BN1 | B, C, D, E, F", new PathPlannerAuto("C5, BN1, { B, C, D, E, F}"));

		// 6 Coral
		autoChooser.addOption("C6 | UN1 | A, B, I, J, K, L", new PathPlannerAuto("C6, UN1, {A, B, I, J, K, L}"));
		autoChooser.addOption("C6 | BN1 | A, B, C, D, E, F", new PathPlannerAuto("C6, BN1, {A, B, C, D, E, F}"));

		SmartDashboard.putData("Auto Chooser", autoChooser);
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
