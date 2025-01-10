package frc.robot.Subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseContainer {

	public static final class PosePair {

		public Pose2d leftPose;
		public Pose2d rightPose;

		public PosePair(Pose2d leftPose, Pose2d rightPose) {
			this.leftPose = leftPose;
			this.rightPose = rightPose;
		}
	}

	private PosePair[] reefSides;
	private PosePair coralStationPair;

	public PoseContainer(PosePair[] reefSides, PosePair coralStationPair) {
		this.reefSides = reefSides;
		this.coralStationPair = coralStationPair;
	}

	public PosePair getCoralStationPair() {
		return coralStationPair;
	}

	public PosePair getReefSides(int reefNumber) {
		return reefSides[reefNumber];
	}
}
