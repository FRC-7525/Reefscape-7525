package frc.robot.subsystems.Vision;

import static frc.robot.GlobalConstants.*;
import static frc.robot.GlobalConstants.Vision.*;

import frc.robot.pioneersLib.misc.VisionUtil;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.Drive.Drive;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Vision extends Subsystem<VisionStates> {

	private VisionIO io;
	private Drive drive;

	private static Vision instance;

	private Vision(VisionIO io, Drive drive) {
		super("Vision", VisionStates.ON);
		this.io = io;
		this.drive = drive;
	}

	public static Vision getInstance() {
		if (instance == null) {
			VisionIO visionIO =
				switch (ROBOT_MODE) {
					case REAL -> new VisionIOReal();
					case SIM -> new VisionIOSim();
					case TESTING -> new VisionIO() {};
					case REPLAY -> new VisionIOSim();
				};
			instance = new Vision(visionIO, Drive.getInstance());
		}
		return instance;
	}

	@Override
	public void runState() {
		if (getState().getVisionEnabled()) {
			io.setStrategy(getState().getStrategy());
			io.updateRobotPose(drive.getPose());

			Optional<EstimatedRobotPose> frontPose = io.getFrontPoseEstimation();
			if (io.getFrontPoseEstimation().isPresent()) {
				drive.addVisionMeasurement(
					frontPose.get().estimatedPose.toPose2d(),
					frontPose.get().timestampSeconds,
					VisionUtil.getEstimationStdDevs(frontPose.get(), FRONT_RESOLUTION)
				);
			}

			Optional<EstimatedRobotPose> backPose = io.getBackPoseEstimation();
			if (backPose.isPresent()) {
				drive.addVisionMeasurement(
					backPose.get().estimatedPose.toPose2d(),
					backPose.get().timestampSeconds,
					VisionUtil.getEstimationStdDevs(backPose.get(), BACK_RESOLUTION)
				);
			}
		}
	}
}