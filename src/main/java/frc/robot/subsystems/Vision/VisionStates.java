<<<<<<< HEAD
package frc.robot.Subsystems.Vision;

import org.team7525.subsystem.SubsystemStates;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
=======
package frc.robot.subsystems.Vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team7525.subsystem.SubsystemStates;
>>>>>>> 6170d84f776a200f651043c1f12fc5104016ffe0

public enum VisionStates implements SubsystemStates {
	ON("Vision On", true, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
	LOWEST_AMBIGUITY("Lowest Ambiguity", true, PoseStrategy.LOWEST_AMBIGUITY),
	OFF("Vision Off", false, PoseStrategy.CLOSEST_TO_LAST_POSE);

	VisionStates(String stateString, boolean visionEnabled, PoseStrategy strategy) {
		this.visionEnabled = visionEnabled;
		this.strategy = strategy;
	}

	private boolean visionEnabled;
	private PoseStrategy strategy;
	private String stateString;

	public boolean getVisionEnabled() {
		return visionEnabled;
	}

	public PoseStrategy getStrategy() {
		return strategy;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
<<<<<<< HEAD
}
=======
}
>>>>>>> 6170d84f776a200f651043c1f12fc5104016ffe0
