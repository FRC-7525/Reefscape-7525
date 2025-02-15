package frc.robot.Subsystems.Vision;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team7525.misc.VisionUtil;
import org.team7525.subsystem.Subsystem;

public class Vision extends Subsystem<VisionStates> {
	private static Vision instance;
	private PhotonCamera backCamera;
	private PhotonCamera frontCamera;
	private PhotonPoseEstimator backEstimator;
	private PhotonPoseEstimator frontEstimator;

	public static Vision getInstance() {
		if (instance == null) {
			instance = new Vision();
		}
		return instance;
	}

	public Optional<EstimatedRobotPose> getBackPoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (PhotonPipelineResult change : backCamera.getAllUnreadResults()) {
			pose = backEstimator.update(change);
			// System.out.print(change);
			// System.out.print(pose);
		}
		return pose;
	}

	// public Optional<EstimatedRobotPose> getFrontPoseEstimation() {
	// 	Optional<EstimatedRobotPose> pose = Optional.empty();
	// 	for (PhotonPipelineResult change : frontCamera.getAllUnreadResults()) {
	// 		pose = frontEstimator.update(change);
	// 	}
	// 	return pose;
	// }

	private Vision() {
		super("Vision", VisionStates.ON);
		backCamera = new PhotonCamera("Back Camera");
		frontCamera = new PhotonCamera("Front Camera");

		// Pose estimators :/
		frontEstimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_FRONT_CAMERA);
		backEstimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_BACK_CAMERA);
	}

	

	@Override
	public void runState() {
		Optional<EstimatedRobotPose> backPose = getBackPoseEstimation();
		// Optional<EstimatedRobotPose> frontPose = getFrontPoseEstimation();

		Logger.recordOutput("Vision/BackCamConnected", backCamera.isConnected());
		Logger.recordOutput("Vision/FrontCamConnected",frontCamera.isConnected());
		Logger.recordOutput("Vision/state", getState().getStateString());
		if (backPose.isPresent()) {
			Logger.recordOutput("Vision/BackPose",backPose.get().estimatedPose.toPose2d()); 
			Logger.recordOutput("Vision/FrontTargets", 		backPose.get().targetsUsed.size());
		}
		// if (frontPose.isPresent()) { 
		// 	Logger.recordOutput("Vision/FrontPose", frontPose.get().estimatedPose.toPose2d());
		// 	Logger.recordOutput("Vision/BackTargets", frontPose.get().targetsUsed.size());
		// }
	}
}
