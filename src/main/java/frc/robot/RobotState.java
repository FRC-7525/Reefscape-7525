package frc.robot;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.AutoAlign.AATypeManager.AATypeManager;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.Passthrough.Passthrough;
import frc.robot.Subsystems.Vision.Vision;

public class RobotState {

	// Big static class of all the subsystems, less messy than just using singleton from all of them
	// and more dynamic than dependency injection (which we should probably use)

	private static RobotState instance;

	private static SubsystemProvider<?> driveProvider, ledProvider, coralerProvider, elevatorProvider, autoAlignProvider, passthroughProvider, frontVisionProvider, backVisionProvider, aaTypeManager;

	@FunctionalInterface
	public interface SubsystemProvider<T> {
		T get();
	}

	private RobotState() {}

	public RobotState getState() {
		if (instance == null) {
			instance = new RobotState();
		}
		return this;
	}

	// if u want to access subsystems directly
	public static Drive getDrive() {
		if (driveProvider == null) {
			driveProvider = Drive::getInstance;
		}
		return (Drive) driveProvider.get();
	}

	public static LED getLED() {
		if (ledProvider == null) {
			ledProvider = LED::getInstance;
		}
		return (LED) ledProvider.get();
	}

	public static Coraler getCoraler() {
		if (coralerProvider == null) {
			coralerProvider = Coraler::getInstance;
		}
		return (Coraler) coralerProvider.get();
	}

	public static Elevator getElevator() {
		if (elevatorProvider == null) {
			elevatorProvider = Elevator::getInstance;
		}
		return (Elevator) elevatorProvider.get();
	}

	public static AutoAlign getAutoAlign() {
		if (autoAlignProvider == null) {
			autoAlignProvider = AutoAlign::getInstance;
		}
		return (AutoAlign) autoAlignProvider.get();
	}

	public static Passthrough getPassthrough() {
		if (passthroughProvider == null) {
			passthroughProvider = Passthrough::getInstance;
		}
		return (Passthrough) passthroughProvider.get();
	}

	public static Vision getFrontVision() {
		if (frontVisionProvider == null) {
			frontVisionProvider = () -> new Vision("Front Vision", visionMeasurment -> {}, ROBOT_MODE == RobotMode.REAL ? FRONT_REAL_IOS : FRONT_SIM_IOS);
		}
		return (Vision) frontVisionProvider.get();
	}

	public static Vision getBackVision() {
		if (backVisionProvider == null) {
			backVisionProvider = () -> new Vision("Back Vision", visionMeasurment -> getDrive().addVisionMeasurement(visionMeasurment.pose(), visionMeasurment.timestamp(), visionMeasurment.standardDev()), ROBOT_MODE == RobotMode.REAL ? BACK_REAL_IOS : BACK_SIM_IOS);
		}
		return (Vision) backVisionProvider.get();
	}

	public static AATypeManager getAATypeManager() {
		if (aaTypeManager == null) {
			aaTypeManager = AATypeManager::getInstance;
		}
		return (AATypeManager) aaTypeManager.get();
	}

	// General util, less messy
	public static Pose2d getPose() {
		return getDrive().getPose();
	}
}
