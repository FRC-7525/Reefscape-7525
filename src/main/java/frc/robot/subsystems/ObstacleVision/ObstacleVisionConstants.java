package frc.robot.Subsystems.ObstacleVision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;

public class ObstacleVisionConstants {
    private static final Time EXPIRATION_TIME = Seconds.of(3);

    public static class ObstaclePoseObservation {
        double timestamp;
        Pose2d observedPose;

        public ObstaclePoseObservation(double timestamp, Pose2d observedPose) {
            this.timestamp = timestamp;
            this.observedPose = observedPose;
        }
    }

}
