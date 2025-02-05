package frc.robot.Subsystems.Vision;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import frc.robot.Subsystems.Drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.team7525.misc.VisionUtil;
import org.team7525.subsystem.Subsystem;

public class Vision extends Subsystem<VisionStates> {

	private VisionIO io;
	private Drive drive;

	private static Vision instance;

	private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

	private Vision() {
		super(SUBSYSTEM_NAME, VisionStates.ON);
		this.io = switch (ROBOT_MODE) {
			case REAL -> new VisionIOReal();
			case SIM -> new VisionIOSim();
			case TESTING -> new VisionIO() {};
			case REPLAY -> new VisionIOSim();
		};
		this.drive = Drive.getInstance();
	}

	public static Vision getInstance() {
		if (instance == null) {
			instance = new Vision();
		}
		return instance;
	}

	@Override
	public void runState() {
		Logger.recordOutput(SUBSYSTEM_NAME + "/back cam", ROBOT_TO_BACK_CAMERA);
		Logger.recordOutput(SUBSYSTEM_NAME + "/front cam", ROBOT_TO_FRONT_CAMERA);

		if (getState().getVisionEnabled()) {
			io.setStrategy(getState().getStrategy());
			io.updateRobotPose(drive.getPose());

			Optional<EstimatedRobotPose> frontPose = io.getFrontPoseEstimation();
			if (frontPose.isPresent()) {
				drive.addVisionMeasurement(frontPose.get().estimatedPose.toPose2d(), frontPose.get().timestampSeconds, VisionUtil.getEstimationStdDevs(frontPose.get(), FRONT_RESOLUTION));
			}

			Optional<EstimatedRobotPose> backPose = io.getBackPoseEstimation();
			if (backPose.isPresent()) {
				drive.addVisionMeasurement(backPose.get().estimatedPose.toPose2d(), backPose.get().timestampSeconds, VisionUtil.getEstimationStdDevs(backPose.get(), BACK_RESOLUTION));
			}
		}
		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);
	}
}
