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
	L1("Driving to Reef Left 1", PosePair.of(new Pose2d(14.404, 3.884, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 4.163, Rotation2d.fromDegrees(0))), Meters.of(.5)),
	R1("Driving to Reef Right 1", PosePair.of(new Pose2d(14.406, 4.206, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 3.828, Rotation2d.fromDegrees(0))), Meters.of(0.5)),
	L2("Driving to Reef Left 2", PosePair.of(new Pose2d(13.857, 5.115, Rotation2d.fromDegrees(240)), new Pose2d(3.686, 2.925, Rotation2d.fromDegrees(60))), Meters.of(0.5)),
	R2("Driving to Reef Right 2", PosePair.of(new Pose2d(13.567, 5.281, Rotation2d.fromDegrees(240)), new Pose2d(3.958, 2.760, Rotation2d.fromDegrees(60))), Meters.of(0.5)),
	L3("Driving to Reef Left 3", PosePair.of(new Pose2d(12.512, 5.258, Rotation2d.fromDegrees(300)), new Pose2d(5.038, 2.776, Rotation2d.fromDegrees(120))), Meters.of(0.5)),
	R3("Driving to Reef Right 3", PosePair.of(new Pose2d(12.228, 5.094, Rotation2d.fromDegrees(300)), new Pose2d(5.316, 2.934, Rotation2d.fromDegrees(120))), Meters.of(0.5)),
	L4("Driving to Reef Left 4", PosePair.of(new Pose2d(11.722, 4.160, Rotation2d.fromDegrees(0)), new Pose2d(5.845, 3.884, Rotation2d.fromDegrees(180))), Meters.of(0.5)),
	R4("Driving to Reef Right 4", PosePair.of(new Pose2d(11.720, 3.831, Rotation2d.fromDegrees(0)), new Pose2d(5.846, 4.208, Rotation2d.fromDegrees(180))), Meters.of(0.5)),
	L5("Driving to Reef Left 5", PosePair.of(new Pose2d(12.272, 2.934, Rotation2d.fromDegrees(60)), new Pose2d(5.285, 5.132, Rotation2d.fromDegrees(240))), Meters.of(0.5)),
	R5("Driving to Reef Right 5", PosePair.of(new Pose2d(12.555, 2.771, Rotation2d.fromDegrees(60)), new Pose2d(5.007, 5.291, Rotation2d.fromDegrees(240))), Meters.of(0.5)),
	L6("Driving to Reef Left 6", PosePair.of(new Pose2d(13.603, 2.790, Rotation2d.fromDegrees(120)), new Pose2d(3.935, 5.27, Rotation2d.fromDegrees(300))), Meters.of(0.5)),
	R6("Driving to Reef Right 6", PosePair.of(new Pose2d(13.891, 2.955, Rotation2d.fromDegrees(120)), new Pose2d(3.66, 5.11, Rotation2d.fromDegrees(300))), Meters.of(0.5)),

	L1_1("Driving to Reef 1 for L1", PosePair.of(new Pose2d(14.39824, 4.0125, Rotation2d.fromDegrees(187)), new Pose2d(3.15, 4.013, Rotation2d.fromDegrees(7))), Meters.of(0.5)),
	L1_2("Driving to Reef 2 for L1", PosePair.of(new Pose2d(13.72851, 5.1725, Rotation2d.fromDegrees(247)), new Pose2d(3.81973, 2.8525, Rotation2d.fromDegrees(67))), Meters.of(0.5)),
	L1_3("Driving to Reef 3 for L1", PosePair.of(new Pose2d(12.38907, 5.1725, Rotation2d.fromDegrees(307)), new Pose2d(5.15918, 2.8525, Rotation2d.fromDegrees(127))), Meters.of(0.5)),
	L1_4("Driving to Reef 4 for L1", PosePair.of(new Pose2d(11.71934, 4.0125, Rotation2d.fromDegrees(7)), new Pose2d(5.8289, 4.0125, Rotation2d.fromDegrees(187))), Meters.of(0.5)),
	L1_5("Driving to Reef 5 for L1", PosePair.of(new Pose2d(12.38907, 2.8525, Rotation2d.fromDegrees(67)), new Pose2d(5.15918, 5.1725, Rotation2d.fromDegrees(247))), Meters.of(0.5)),
	L1_6("Driving to Reef 6 for L1", PosePair.of(new Pose2d(13.72851, 2.8525, Rotation2d.fromDegrees(127)), new Pose2d(3.81973, 5.1725, Rotation2d.fromDegrees(307))), Meters.of(0.5)),
	//Blue Alliance (1.472, 0.847) Red Alliance (16.331, 7.004)
	RIGHT_SOURCE("Driving to Source Right", PosePair.of(new Pose2d(16.519, 7.220, Rotation2d.fromDegrees(-126)), new Pose2d(0.983, 0.818, Rotation2d.fromDegrees(54))), Meters.of(1)),
	//Blue Alliance (1.511, 7.194) Red Alliance (16.263, 0.983)
	LEFT_SOURCE("Driving to Source Left", PosePair.of(new Pose2d(16.519, 0.818, Rotation2d.fromDegrees(126)), new Pose2d(1.079, 7.244, Rotation2d.fromDegrees(-54))), Meters.of(1));

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
// OFF("OFF, No Aligning!", PosePair.of(new Pose2d(1, 1, new Rotation2d()), new Pose2d(2, 1, new Rotation2d())), Meters.of(0.0)),
// L1("Driving to Reef Left 1", PosePair.of(new Pose2d(14.409, 3.869, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 4.163, Rotation2d.fromDegrees(0))), Meters.of(.5)),
// R1("Driving to Reef Right 1", PosePair.of(new Pose2d(14.407, 4.197, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 3.828, Rotation2d.fromDegrees(0))), Meters.of(0.5)),
// L2("Driving to Reef Left 2", PosePair.of(new Pose2d(13.869, 5.105, Rotation2d.fromDegrees(240)), new Pose2d(3.686, 2.925, Rotation2d.fromDegrees(60))), Meters.of(0.5)),
// R2("Driving to Reef Right 2", PosePair.of(new Pose2d(13.546, 5.236, Rotation2d.fromDegrees(240)), new Pose2d(3.958, 2.760, Rotation2d.fromDegrees(60))), Meters.of(0.5)),
// L3("Driving to Reef Left 3", PosePair.of(new Pose2d(12.540, 5.209, Rotation2d.fromDegrees(300)), new Pose2d(5.038, 2.776, Rotation2d.fromDegrees(120))), Meters.of(0.5)),
// R3("Driving to Reef Right 3", PosePair.of(new Pose2d(12.253, 5.048, Rotation2d.fromDegrees(300)), new Pose2d(5.316, 2.934, Rotation2d.fromDegrees(120))), Meters.of(0.5)),
// L4("Driving to Reef Left 4", PosePair.of(new Pose2d(11.717, 4.156, Rotation2d.fromDegrees(0)), new Pose2d(5.845, 3.884, Rotation2d.fromDegrees(180))), Meters.of(0.5)),
// R4("Driving to Reef Right 4", PosePair.of(new Pose2d(11.717, 3.828, Rotation2d.fromDegrees(0)), new Pose2d(5.846, 4.208, Rotation2d.fromDegrees(180))), Meters.of(0.5)),
// L5("Driving to Reef Left 5", PosePair.of(new Pose2d(12.271, 2.928, Rotation2d.fromDegrees(60)), new Pose2d(5.285, 5.132, Rotation2d.fromDegrees(240))), Meters.of(0.5)),
// R5("Driving to Reef Right 5", PosePair.of(new Pose2d(12.558, 2.764, Rotation2d.fromDegrees(60)), new Pose2d(5.007, 5.291, Rotation2d.fromDegrees(240))), Meters.of(0.5)),
// L6("Driving to Reef Left 6", PosePair.of(new Pose2d(13.602, 2.793, Rotation2d.fromDegrees(120)), new Pose2d(3.935, 5.27, Rotation2d.fromDegrees(300))), Meters.of(0.5)),
