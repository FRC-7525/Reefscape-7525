package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MASS;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import static frc.robot.Subsystems.Drive.TunerConstants.*;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

/**
 * This class represents the simulated input/output for the drive subsystem.
 * It extends the DriveIOReal class and provides additional functionality for simulation.
 */
public class DriveIOSim extends DriveIOReal {

	private SwerveModuleSimulationConfig swerveModConfigs = new SwerveModuleSimulationConfig(DCMotor.getKrakenX60Foc(1), DCMotor.getFalcon500(1), TunerConstants.FrontLeft.DriveMotorGearRatio, TunerConstants.FrontLeft.SteerMotorGearRatio, Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage), Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage), Meters.of(TunerConstants.FrontLeft.WheelRadius), KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia), 1.2);


	@SuppressWarnings("unchecked")
	private final DriveTrainSimulationConfig driveConfig = new DriveTrainSimulationConfig(ROBOT_MASS,
	Meters.of(.826),
	Meters.of(.826),
	Meters.of(0.546),
	Meters.of(0.546),
	() -> new GyroSimulation(SIM_UPDATE_TIME, SIM_UPDATE_TIME),
	() -> {return new SwerveModuleSimulation(swerveModConfigs);});


	private double lastSimTime;
	private Notifier simNotifier;
	private SwerveDriveSimulation mapleSwerveDrive = null;

	/**
	 * Constructs a new DriveIOSim object.
	 * It initializes the superclass and starts the simulation thread.
	 */
	public DriveIOSim() {
		startSimThread();
	}

	private void startSimThread() {
		lastSimTime = Utils.getCurrentTimeSeconds();

		mapleSwerveDrive = new SwerveDriveSimulation(null, null);

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - lastSimTime;
			lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			getDrive().updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		simNotifier.startPeriodic(SIM_UPDATE_TIME);
	}
	
}
