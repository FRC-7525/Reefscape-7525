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

	L1("Driving to Reef Left 1", PosePair.of(new Pose2d(14.387, 3.828, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 4.163, Rotation2d.fromDegrees(0))), Meters.of(.5)),

	R1("Driving to Reef Right 1", PosePair.of(new Pose2d(14.387, 4.163, Rotation2d.fromDegrees(180)), new Pose2d(3.209, 3.828, Rotation2d.fromDegrees(0))), Meters.of(0.5)),

	L2("Driving to Reef Left 2", PosePair.of(new Pose2d(13.9181622934, 5.03585272828, Rotation2d.fromDegrees(240)), new Pose2d(3.730493783166557, 2.985548271724216, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	R2("Driving to Reef Right 2", PosePair.of(new Pose2d(13.6280437832, 5.20335272828, Rotation2d.fromDegrees(240)), new Pose2d(4.020612293434344, 2.818048271724216, Rotation2d.fromDegrees(60))), Meters.of(0.5)),

	L3("Driving to Reef Left 3", PosePair.of(new Pose2d(12.6377122934, 5.23375372828, Rotation2d.fromDegrees(300)), new Pose2d(5.010943783166557, 2.848449271724216, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	R3("Driving to Reef Right 3", PosePair.of(new Pose2d(12.3475937832, 5.06625372828, Rotation2d.fromDegrees(300)), new Pose2d(5.301062293434344, 3.015949271724216, Rotation2d.fromDegrees(120))), Meters.of(0.5)),

	L4("Driving to Reef Left 4", PosePair.of(new Pose2d(11.8261, 4.223802, Rotation2d.fromDegrees(0)), new Pose2d(5.7699, 3.888802, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	R4("Driving to Reef Right 4", PosePair.of(new Pose2d(11.8261, 3.888802, Rotation2d.fromDegrees(0)), new Pose2d(5.7699, 4.223802, Rotation2d.fromDegrees(180))), Meters.of(0.5)),

	L5("Driving to Reef Left 5", PosePair.of(new Pose2d(12.2949377066, 3.01594927172, Rotation2d.fromDegrees(60)), new Pose2d(5.248406216833443, 5.066253728275784, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	R5("Driving to Reef Right 5", PosePair.of(new Pose2d(12.5850562168, 2.84844927172, Rotation2d.fromDegrees(60)), new Pose2d(4.958287706565656, 5.233753728275784, Rotation2d.fromDegrees(240))), Meters.of(0.5)),

	L6("Driving to Reef Left 6", PosePair.of(new Pose2d(13.5753877066, 2.81804827172, Rotation2d.fromDegrees(120)), new Pose2d(3.967956216833443, 5.203352728275784, Rotation2d.fromDegrees(300))), Meters.of(0.5)),

	R6("Driving to Reef Right 6", PosePair.of(new Pose2d(13.8655062168, 2.98554827172, Rotation2d.fromDegrees(120)), new Pose2d(3.677837706565656, 5.035852728275784, Rotation2d.fromDegrees(300))), Meters.of(0.5)),

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
