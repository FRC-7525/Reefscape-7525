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

	L1("Driving to Reef Left 1", PosePair.of(new Pose2d(14.409, 3.869, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 4.163, Rotation2d.fromDegrees(0))), Meters.of(.5)),

	R1("Driving to Reef Right 1", PosePair.of(new Pose2d(14.407, 4.197, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 3.828, Rotation2d.fromDegrees(0))), Meters.of(0.5)),

	L2("Driving to Reef Left 2", PosePair.of(new Pose2d(13.829, 5.074, Rotation2d.fromDegrees(240)), new Pose2d(3.731, 2.971, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	R2("Driving to Reef Right 2", PosePair.of(new Pose2d(13.546, 5.236, Rotation2d.fromDegrees(240)), new Pose2d(4.021, 2.803, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	L3("Driving to Reef Left 3", PosePair.of(new Pose2d(12.540, 5.209, Rotation2d.fromDegrees(300)), new Pose2d(5.010, 2.832, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	R3("Driving to Reef Right 3", PosePair.of(new Pose2d(12.253, 5.048, Rotation2d.fromDegrees(300)), new Pose2d(5.291, 2.998, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	L4("Driving to Reef Left 4", PosePair.of(new Pose2d(11.717, 4.156, Rotation2d.fromDegrees(0)), new Pose2d(5.845, 3.884, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	R4("Driving to Reef Right 4", PosePair.of(new Pose2d(11.717, 3.828, Rotation2d.fromDegrees(0)), new Pose2d(5.846, 4.208, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	L5("Driving to Reef Left 5", PosePair.of(new Pose2d(12.271, 2.928, Rotation2d.fromDegrees(60)), new Pose2d(5.285, 5.132, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	R5("Driving to Reef Right 5", PosePair.of(new Pose2d(12.558, 2.764, Rotation2d.fromDegrees(60)), new Pose2d(5.007, 5.291, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	L6("Driving to Reef Left 6", PosePair.of(new Pose2d(13.602, 2.793, Rotation2d.fromDegrees(120)), new Pose2d(3.935, 5.27, Rotation2d.fromDegrees(300))), Meters.of(0.5)),

	R6("Driving to Reef Right 6", PosePair.of(new Pose2d(13.912, 2.964, Rotation2d.fromDegrees(120)), new Pose2d(3.66, 5.11, Rotation2d.fromDegrees(300))), Meters.of(0.5)),

	//Blue Alliance (1.110, 1.03) Red Alliance (16.303. 7.104)
	RIGHT_SOURCE("Driving to Source Right", PosePair.of(new Pose2d(16.916, 7.691, Rotation2d.fromDegrees(-126)), new Pose2d(0.878, 0.632, Rotation2d.fromDegrees(54))), Meters.of(1)),
	//Blue alliance (1.236, 7.104) Red Alliance (16.418, 1.006)
	LEFT_SOURCE("Driving to Source Left", PosePair.of(new Pose2d(17.004, 0.359, Rotation2d.fromDegrees(126)), new Pose2d(0.614, 0.32, Rotation2d.fromDegrees(306))), Meters.of(1));

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
