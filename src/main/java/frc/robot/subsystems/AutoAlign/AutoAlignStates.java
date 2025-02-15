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

	L1("Driving to Reef Left 1", PosePair.of(new Pose2d(14.348, 3.81, Rotation2d.fromDegrees(180)), new Pose2d(3.2, 4.247, Rotation2d.fromDegrees(0))), Meters.of(.5)),

	R1("Driving to Reef Right 1", PosePair.of(new Pose2d(14.345, 4.130, Rotation2d.fromDegrees(180)), new Pose2d(3.2, 3.923, Rotation2d.fromDegrees(0))), Meters.of(0.5)),

	L2("Driving to Reef Left 2", PosePair.of(new Pose2d(13.892, 5.026, Rotation2d.fromDegrees(240)), new Pose2d(3.658, 3.028, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	R2("Driving to Reef Right 2", PosePair.of(new Pose2d(13.612, 5.191, Rotation2d.fromDegrees(240)), new Pose2d(2.864, 3.946, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	L3("Driving to Reef Left 3", PosePair.of(new Pose2d(12.605, 5.243, Rotation2d.fromDegrees(300)), new Pose2d(4.934, 2.802, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	R3("Driving to Reef Right 3", PosePair.of(new Pose2d(12.330, 5.082, Rotation2d.fromDegrees(300)), new Pose2d(5.202, 2.994, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	L4("Driving to Reef Left 4", PosePair.of(new Pose2d(11.779, 4.243, Rotation2d.fromDegrees(0)), new Pose2d(5.77, 3.805, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	R4("Driving to Reef Right 4", PosePair.of(new Pose2d(11.780, 3.914, Rotation2d.fromDegrees(0)), new Pose2d(5.77, 4.134, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	L5("Driving to Reef Left 5", PosePair.of(new Pose2d(12.238, 3.021, Rotation2d.fromDegrees(60)), new Pose2d(5.316, 5.026, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	R5("Driving to Reef Right 5", PosePair.of(new Pose2d(12.520, 2.858, Rotation2d.fromDegrees(60)), new Pose2d(5.043, 5.186, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	L6("Driving to Reef Left 6", PosePair.of(new Pose2d(13.517, 2.805, Rotation2d.fromDegrees(120)), new Pose2d(2, 12, Rotation2d.fromDegrees(180))), Meters.of(0.5)), // TODO add red poses for last 2

	R6("Driving to Reef Right 6", PosePair.of(new Pose2d(13.792, 2.967, Rotation2d.fromDegrees(120)), new Pose2d(2, 13, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	RIGHT_SOURCE("Driving to Source Right", PosePair.of(new Pose2d(1, 14, Rotation2d.fromDegrees(180)), new Pose2d(2, 14, Rotation2d.fromDegrees(180))), Meters.of(0.5)),
	LEFT_SOURCE("Driving to Source Left", PosePair.of(new Pose2d(1, 15, Rotation2d.fromDegrees(180)), new Pose2d(2, 15, Rotation2d.fromDegrees(180))), Meters.of(0.5));

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
