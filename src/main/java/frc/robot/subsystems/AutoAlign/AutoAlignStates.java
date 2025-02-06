package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.team7525.subsystem.SubsystemStates;

public enum AutoAlignStates implements SubsystemStates {
	// TODO: Have real states and tune distance for close AA
	OFF("OFF, No Aligning!", PosePair.of(new Pose2d(1, 1, new Rotation2d()), new Pose2d(2, 1, new Rotation2d())), Meters.of(0.0)),
	L1("Driving to Reef Left 1", PosePair.of(new Pose2d(1, 2, new Rotation2d()), new Pose2d(2, 2, new Rotation2d())), Meters.of(1.5)),
	R1("Driving to Reef Right 1", PosePair.of(new Pose2d(1, 3, new Rotation2d()), new Pose2d(2, 3, new Rotation2d())), Meters.of(1.5)),
	L2("Driving to Reef Left 2", PosePair.of(new Pose2d(1, 4, new Rotation2d()), new Pose2d(2, 4, new Rotation2d())), Meters.of(1.5)),
	R2("Driving to Reef Right 2", PosePair.of(new Pose2d(1, 5, new Rotation2d()), new Pose2d(2, 5, new Rotation2d())), Meters.of(1.5)),
	L3("Driving to Reef Left 3", PosePair.of(new Pose2d(1, 6, new Rotation2d()), new Pose2d(2, 6, new Rotation2d())), Meters.of(1.5)),
	R3("Driving to Reef Right 3", PosePair.of(new Pose2d(1, 7, new Rotation2d()), new Pose2d(2, 7, new Rotation2d())), Meters.of(1.5)),
	L4("Driving to Reef Left 4", PosePair.of(new Pose2d(1, 8, new Rotation2d()), new Pose2d(2, 8, new Rotation2d())), Meters.of(1.5)),
	R4("Driving to Reef Right 4", PosePair.of(new Pose2d(1, 9, new Rotation2d()), new Pose2d(2, 9, new Rotation2d())), Meters.of(1.5)),
	L5("Driving to Reef Left 5", PosePair.of(new Pose2d(1, 10, new Rotation2d()), new Pose2d(2, 10, new Rotation2d())), Meters.of(1.5)),
	R5("Driving to Reef Right 5", PosePair.of(new Pose2d(1, 11, new Rotation2d()), new Pose2d(2, 11, new Rotation2d())), Meters.of(1.5)),
	L6("Driving to Reef Left 6", PosePair.of(new Pose2d(1, 12, new Rotation2d()), new Pose2d(2, 12, new Rotation2d())), Meters.of(1.5)),
	R6("Driving to Reef Right 6", PosePair.of(new Pose2d(1, 13, new Rotation2d()), new Pose2d(2, 13, new Rotation2d())), Meters.of(1.5)),
	RIGHT_SOURCE("Driving to Source Right", PosePair.of(new Pose2d(1, 14, new Rotation2d()), new Pose2d(2, 14, new Rotation2d())), Meters.of(1.5)),
	LEFT_SOURCE("Driving to Source Left", PosePair.of(new Pose2d(1, 15, new Rotation2d()), new Pose2d(2, 15, new Rotation2d())), Meters.of(1.5));

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
