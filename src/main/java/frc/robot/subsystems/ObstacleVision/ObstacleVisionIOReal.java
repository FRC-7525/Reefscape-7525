package frc.robot.Subsystems.ObstacleVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Subsystems.Drive.Drive;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

public class ObstacleVisionIOReal implements ObstacleVisionIO {

	private PhotonCamera camera;
	private Transform3d robotToCamera;
	private ArrayList<Pose2d> obstacleList;

	public ObstacleVisionIOReal(String cameraName, Transform3d robotToCamera) {
		camera = new PhotonCamera(cameraName);
		this.robotToCamera = robotToCamera;
		obstacleList = new ArrayList<Pose2d>();
	}

	public ArrayList<Pose2d> getObstaclePoses() {
		for (var result : camera.getAllUnreadResults()) {
			for (var obstacle : result.getTargets()) {
				Transform3d cameraToObject = obstacle.getBestCameraToTarget();
				Pose2d robotPose = Drive.getInstance().getPose();

				Translation3d transformRobotRelative = cameraToObject.getTranslation().rotateBy(robotToCamera.getRotation().unaryMinus());
				Pose2d obstaclePose = robotPose.transformBy(new Transform2d(transformRobotRelative.toTranslation2d(), cameraToObject.getRotation().toRotation2d()));

				obstacleList.add(obstaclePose);
			}
		}
		return obstacleList;
	}
}
