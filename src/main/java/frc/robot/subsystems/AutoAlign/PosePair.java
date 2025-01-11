package frc.robot.Subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;

public final class PosePair {

	private Pose2d redPose;
	private Pose2d bluePose;

	public PosePair(Pose2d redPose, Pose2d bluePose) {
		this.redPose = redPose;
		this.bluePose = bluePose;
	}

	public Pose2d getRedPose() {
		return redPose;
	}

	public Pose2d getBluePose() {
		return bluePose;
	}
}
