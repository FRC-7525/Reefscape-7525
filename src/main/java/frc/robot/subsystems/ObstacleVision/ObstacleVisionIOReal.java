package frc.robot.Subsystems.ObstacleVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.ObstacleVision.ObstacleVisionConstants.ObstaclePoseObservation;

public class ObstacleVisionIOReal implements ObstacleVisionIO {

	private PhotonCamera camera;
	private Transform3d robotToCamera;
	private ArrayList<ObstaclePoseObservation> obstacleList;

    public ObstacleVisionIOReal(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
        obstacleList = new ArrayList<ObstaclePoseObservation>();
    }

    public ArrayList<ObstaclePoseObservation> getObstaclePoses() {
        obstacleList.clear();
        for (var result : camera.getAllUnreadResults()) {
            for (var obstacle : result.getTargets()) {
                Transform3d cameraToObject = obstacle.getBestCameraToTarget();
                Pose2d robotPose = Drive.getInstance().getPose();

                Translation2d fieldRelativeObjectTranslation = cameraToObject.plus(robotToCamera).getTranslation().toTranslation2d()
                .rotateBy(robotPose.getRotation())
                .rotateBy(robotToCamera.getRotation().toRotation2d());
                Pose2d obstaclePose = new Pose2d(fieldRelativeObjectTranslation.plus(robotPose.getTranslation()), new Rotation2d());

                obstacleList.add(new ObstaclePoseObservation(result.getTimestampSeconds(), obstaclePose));
                Logger.recordOutput("ObjectVision", obstaclePose);
            }
        }
        return obstacleList;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }
}
