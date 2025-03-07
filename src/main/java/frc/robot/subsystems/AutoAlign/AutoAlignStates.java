package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.team7525.subsystem.SubsystemStates;

public enum AutoAlignStates implements SubsystemStates {
	OFF("OFF, No Aligning!", PosePair.of(new Pose2d(1, 1, new Rotation2d()), new Pose2d(2, 1, new Rotation2d())), Meters.of(0.0)),

	L1("Driving to Reef Left 1", PosePair.of(new Pose2d(14.347, 3.88, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 4.163, Rotation2d.fromDegrees(0))), Meters.of(.5)),

	R1("Driving to Reef Right 1", PosePair.of(new Pose2d(14.347, 4.209, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 3.828, Rotation2d.fromDegrees(0))), Meters.of(0.5)),

	L2("Driving to Reef Left 2", PosePair.of(new Pose2d(13.829, 5.074, Rotation2d.fromDegrees(240)), new Pose2d(3.731, 2.971, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	R2("Driving to Reef Right 2", PosePair.of(new Pose2d(13.546, 5.236, Rotation2d.fromDegrees(240)), new Pose2d(4.021, 2.803, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	L3("Driving to Reef Left 3", PosePair.of(new Pose2d(12.540, 5.209, Rotation2d.fromDegrees(300)), new Pose2d(5.010, 2.832, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	R3("Driving to Reef Right 3", PosePair.of(new Pose2d(12.253, 5.048, Rotation2d.fromDegrees(300)), new Pose2d(5.291, 2.998, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	L4("Driving to Reef Left 4", PosePair.of(new Pose2d(11.769, 4.158, Rotation2d.fromDegrees(0)), new Pose2d(5.78, 3.889, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	R4("Driving to Reef Right 4", PosePair.of(new Pose2d(11.769, 3.827, Rotation2d.fromDegrees(0)), new Pose2d(5.78, 4.210, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	L5("Driving to Reef Left 5", PosePair.of(new Pose2d(12.305, 2.971, Rotation2d.fromDegrees(60)), new Pose2d(5.247, 5.077, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	R5("Driving to Reef Right 5", PosePair.of(new Pose2d(12.598, 2.802, Rotation2d.fromDegrees(60)), new Pose2d(4.971, 5.236, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	L6("Driving to Reef Left 6", PosePair.of(new Pose2d(13.582, 2.834, Rotation2d.fromDegrees(120)), new Pose2d(3.969, 5.217, Rotation2d.fromDegrees(300))), Meters.of(0.5)),

	R6("Driving to Reef Right 6", PosePair.of(new Pose2d(13.867, 2.995, Rotation2d.fromDegrees(120)), new Pose2d(3.681, 5.05, Rotation2d.fromDegrees(300))), Meters.of(0.5)),
	
	//Blue Alliance (1.110, 1.03) Red Alliance (16.303. 7.104)
	RIGHT_SOURCE("Driving to Source Right", PosePair.of(new Pose2d(16.916, 7.691, Rotation2d.fromDegrees(-126)), new Pose2d(0.878, 0.632, Rotation2d.fromDegrees(54))), Meters.of(1)),
	//Blue alliance (1.236, 7.104) Red Alliance (16.418, 1.006)
	LEFT_SOURCE("Driving to Source Left", PosePair.of(new Pose2d(17.004, 0.359, Rotation2d.fromDegrees(126)), new Pose2d(0.935, 7.531, Rotation2d.fromDegrees(306))), Meters.of(1));

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
