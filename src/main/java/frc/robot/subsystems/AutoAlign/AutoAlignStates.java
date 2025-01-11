package frc.robot.Subsystems.AutoAlign;

import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.math.geometry.Pose2d;

public enum AutoAlignStates implements SubsystemStates {
	OFF("OFF, No Aligning!", new Pose2d()),
	L1("Driving to Reef Left 1", new Pose2d()),
	R1("Driving to Reef Right 1", new Pose2d()),
	L2("Driving to Reef Left 2", new Pose2d()),
	R2("Driving to Reef Right 2", new Pose2d()),
	L3("Driving to Reef Left 3", new Pose2d()),
	R3("Driving to Reef Right 3", new Pose2d()),
	L4("Driving to Reef Left 4", new Pose2d()),
	R4("Driving to Reef Right 4", new Pose2d()),
	L5("Driving to Reef Left 5", new Pose2d()),
	R5("Driving to Reef Right 5", new Pose2d()),
	L6("Driving to Reef Left 6", new Pose2d()),
	R6("Driving to Reef Right 6", new Pose2d()),
	RIGHT_SOURCE("Driving to Source Right", new Pose2d()),
	LEFT_SOURCE("Driving to Source Left", new Pose2d()),;

	AutoAlignStates(String stateString, Pose2d targetPose) {
		this.stateString = stateString;
		this.targetPose = targetPose;
	}

	private String stateString;
	private Pose2d targetPose;

	@Override
	public String getStateString() {
		return stateString;
	}

	public Pose2d getTargetPose() {
		return targetPose;
	}
}
