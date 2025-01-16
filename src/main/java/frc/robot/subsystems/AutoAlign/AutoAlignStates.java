package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.team7525.subsystem.SubsystemStates;

public enum AutoAlignStates implements SubsystemStates {
	// TODO: Have real states and tune distance for close AA
	OFF("OFF, No Aligning!", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(0.0)),
	L1("Driving to Reef Left 1", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	R1("Driving to Reef Right 1", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	L2("Driving to Reef Left 2", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	R2("Driving to Reef Right 2", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	L3("Driving to Reef Left 3", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	R3("Driving to Reef Right 3", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	L4("Driving to Reef Left 4", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	R4("Driving to Reef Right 4", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	L5("Driving to Reef Left 5", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	R5("Driving to Reef Right 5", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	L6("Driving to Reef Left 6", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	R6("Driving to Reef Right 6", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	RIGHT_SOURCE("Driving to Source Right", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5)),
	LEFT_SOURCE("Driving to Source Left", PosePair.of(new Pose2d(), new Pose2d()), Meters.of(1.5));

	AutoAlignStates(String stateString, PosePair targetPose, Distance distanceForCloseAA) {
		this.stateString = stateString;
		this.targetPose = targetPose;
		this.distanceForCloseAA = distanceForCloseAA;
	}

	private String stateString;
	private PosePair targetPose;
	private Distance distanceForCloseAA;

	@Override
	public String getStateString() {
		return stateString;
	}

	public Pose2d getTargetPose() {
		return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? targetPose.getRedPose() : targetPose.getBluePose();
	}

	public Distance getDistanceForCloseAA() {
		return distanceForCloseAA;
	}
}
