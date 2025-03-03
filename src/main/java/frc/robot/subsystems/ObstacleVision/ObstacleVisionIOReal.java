package frc.robot.Subsystems.ObstacleVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
                Transform3d cameraToObject = obstacle.getBestCameraToTarget();
                Pose2d robotPose = Drive.getInstance().getPose();

                // if (robotPose.getTranslation().getDistance(reefPose2d.getTranslation()) > REEF_HITBOX.in(Meters)) continue;

                Translation2d fieldRelativeObjectTranslation = cameraToObject.plus(robotToCamera).getTranslation().toTranslation2d().rotateBy(robotPose.getRotation());
                Pose2d obstaclePose = new Pose2d(fieldRelativeObjectTranslation, new Rotation2d());

                // for (int i = 0; i < obstacleList.size(); i++) {
                //     if (obstacleList.get(i).getTranslation().getDistance(obstaclePose.getTranslation()) < .1) {
                //         alreadyListed = true;
                //         obstacleList.set(i, obstaclePose);
                //         break;
                //     }
                // }

                obstacleList.add(obstaclePose);
                System.out.println("hi");
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
