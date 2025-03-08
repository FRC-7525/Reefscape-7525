package frc.robot.Subsystems.ObstacleVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.ObstacleVision.ObstacleVisionConstants.ObstaclePoseObservation;

public class ObstacleVisionIOSim implements ObstacleVisionIO {

	private PhotonCamera camera;
	private Transform3d robotToCamera;
	private ArrayList<ObstaclePoseObservation> obstacleList;

    public ObstacleVisionIOSim(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
        obstacleList = new ArrayList<ObstaclePoseObservation>();
    }

    public ArrayList<ObstaclePoseObservation> getObstaclePoses() { //TODO Implement further. This is only testing if the transform stuff is correct.
        obstacleList.clear();
        Transform3d cameraToObject = new Transform3d(1, 0, 0, new Rotation3d());
        Pose2d robotPose = Drive.getInstance().getPose();

        // if (robotPose.getTranslation().getDistance(reefPose2d.getTranslation()) > REEF_HITBOX.in(Meters)) continue;

        Translation2d fieldRelativeObjectTranslation = cameraToObject.plus(robotToCamera).getTranslation().toTranslation2d()
            .rotateBy(robotPose.getRotation())
            .rotateBy(robotToCamera.getRotation().toRotation2d());
        Pose2d obstaclePose = new Pose2d(fieldRelativeObjectTranslation.plus(robotPose.getTranslation()), new Rotation2d());


        obstacleList.add(new ObstaclePoseObservation(0, obstaclePose));
        Logger.recordOutput("ObjectVision", obstaclePose);
        return obstacleList;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }
}
