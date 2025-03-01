package frc.robot.Subsystems.ObstacleVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Subsystems.Drive.Drive;

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
                boolean alreadyListed = false;
                Transform3d cameraToObject = obstacle.getBestCameraToTarget();
                Pose2d robotPose = Drive.getInstance().getPose();

                Translation2d fieldRelativeObjectTranslation = cameraToObject.plus(robotToCamera).getTranslation().toTranslation2d().rotateBy(robotPose.getRotation());
                Pose2d obstaclePose = new Pose2d(fieldRelativeObjectTranslation, new Rotation2d());
                

                // Pose2d obstaclePose = robotPose.transformBy(new Transform2d(transformRobotRelative.toTranslation2d().plus(robotToCamera.getTranslation().toTranslation2d()), cameraToObject.getRotation().toRotation2d()));

                for (int i = 0; i < obstacleList.size(); i++) {
                    if (obstacleList.get(i).getTranslation().getDistance(obstaclePose.getTranslation()) > .1) {
                        alreadyListed = true;
                        obstacleList.set(i, obstaclePose);
                        break;
                    }

                    if (true);
                }

                obstacleList.add(obstaclePose);
                Logger.recordOutput("ObjectVision", obstaclePose);
            }
        }
        return obstacleList;
        
    }
}
