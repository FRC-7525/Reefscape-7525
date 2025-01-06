package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.GlobalConstants;

/**
 * This class represents the simulated input/output for the drive subsystem.
 * It extends the DriveIOReal class and provides additional functionality for simulation.
 */
public class DriveIOSim extends DriveIOReal {

	private double lastSimTime;
	private Notifier simNotifier;

	/**
	 * Constructs a new DriveIOSim object.
	 * It initializes the superclass and starts the simulation thread.
	 */
	public DriveIOSim() {
		startSimThread();
	}

	private void startSimThread() {
		lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - lastSimTime;
			lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			getDrive().updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		simNotifier.startPeriodic(GlobalConstants.Drive.SIM_UPDATE_TIME);
	}
}
