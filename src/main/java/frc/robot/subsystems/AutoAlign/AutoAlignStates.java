package frc.robot.Subsystems.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

import org.team7525.subsystem.SubsystemStates;

import static edu.wpi.first.units.Units.*;

public enum AutoAlignStates implements SubsystemStates {
	OFF("OFF, No Aligning!", new Pose2d(), Meters.of(0.0)),
	L1("Driving to Reef Left 1", new Pose2d(), Meters.of(1.5)),
	R1("Driving to Reef Right 1", new Pose2d(), Meters.of(1.5)),
	L2("Driving to Reef Left 2", new Pose2d(), Meters.of(1.5)),
	R2("Driving to Reef Right 2", new Pose2d(), Meters.of(1.5)),
	L3("Driving to Reef Left 3", new Pose2d(), Meters.of(1.5)),
	R3("Driving to Reef Right 3", new Pose2d(), Meters.of(1.5)),
	L4("Driving to Reef Left 4", new Pose2d(), Meters.of(1.5)),
	R4("Driving to Reef Right 4", new Pose2d(), Meters.of(1.5)),
	L5("Driving to Reef Left 5", new Pose2d(), Meters.of(1.5)),
	R5("Driving to Reef Right 5", new Pose2d(), Meters.of(1.5)),
	L6("Driving to Reef Left 6", new Pose2d(), Meters.of(1.5)),
	R6("Driving to Reef Right 6", new Pose2d(), Meters.of(1.5)),
	RIGHT_SOURCE("Driving to Source Right", new Pose2d(), Meters.of(1.5)),
	LEFT_SOURCE("Driving to Source Left", new Pose2d(), Meters.of(1.5));

	AutoAlignStates(String stateString, Pose2d targetPose, Distance distanceForCloseAA) {
		this.stateString = stateString;
		this.targetPose = targetPose;
		this.distanceForCloseAA = distanceForCloseAA;
	}

	private String stateString;
	private Pose2d targetPose;
	private Distance distanceForCloseAA;

	@Override
	public String getStateString() {
		return stateString;
	}

	public Pose2d getTargetPose() {
		return targetPose;
	}

	public Distance getDistanceForCloseAA() {
		return distanceForCloseAA;
	}
}
