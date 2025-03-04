package frc.robot.SubsystemManager;

import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.LED.LEDStates;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public final class SubsystemManagerConstants {

	public static final String SUBSYSTEM_NAME = "Manager";

	public static final double DOWN_DPAD = 180;
	public static final double UP_DPAD = 0;
	public static final double LEFT_DPAD = 270;
	public static final double RIGHT_DPAD = 90;
	public static final double AXIS_RECOGNITION_POINT = 0.8;

	public static final double CORAL_CENTERING_TIME = 0.0;
	public static final double SCORING_TIME = 0.4;

	public static final Map<Integer, ElevatorStates> REEF_SCORING_LEVELS = Map.of(1, ElevatorStates.L1, 2, ElevatorStates.L2, 3, ElevatorStates.L3, 4, ElevatorStates.L4);

	public static class AAReefTarget {

		private int hexagonSide;
		private boolean leftReef;

		private AAReefTarget(int hexagonSide, boolean leftReef) {
			this.hexagonSide = hexagonSide;
			this.leftReef = leftReef;
		}

		public int getHexagonSide() {
			return hexagonSide;
		}

		public boolean isLeftReef() {
			return leftReef;
		}

		public static AAReefTarget of(int hexagonSide, boolean leftReef) {
			return new AAReefTarget(hexagonSide, leftReef);
		}

		@Override
		public boolean equals(Object o) {
			if (this == o) return true;
			if (o == null || getClass() != o.getClass()) return false;
			AAReefTarget that = (AAReefTarget) o;
			return hexagonSide == that.hexagonSide && leftReef == that.leftReef;
		}

		@Override
		public int hashCode() {
			return Objects.hash(hexagonSide, leftReef);
		}

		@Override
		public String toString() {
			return "AAReefTarget{" + "hexagonSide=" + hexagonSide + ", leftReef=" + leftReef + '}';
		}
	}

	public static final Map<AAReefTarget, AutoAlignStates> REEF_TARGET_MAP;

	static {
		final Map<AAReefTarget, AutoAlignStates> hexagonTargetSides = new HashMap<>();
		hexagonTargetSides.put(AAReefTarget.of(1, true), AutoAlignStates.L1);
		hexagonTargetSides.put(AAReefTarget.of(1, false), AutoAlignStates.R1);
		hexagonTargetSides.put(AAReefTarget.of(2, true), AutoAlignStates.L2);
		hexagonTargetSides.put(AAReefTarget.of(2, false), AutoAlignStates.R2);
		hexagonTargetSides.put(AAReefTarget.of(3, true), AutoAlignStates.L3);
		hexagonTargetSides.put(AAReefTarget.of(3, false), AutoAlignStates.R3);
		hexagonTargetSides.put(AAReefTarget.of(4, true), AutoAlignStates.L4);
		hexagonTargetSides.put(AAReefTarget.of(4, false), AutoAlignStates.R4);
		hexagonTargetSides.put(AAReefTarget.of(5, true), AutoAlignStates.L5);
		hexagonTargetSides.put(AAReefTarget.of(5, false), AutoAlignStates.R5);
		hexagonTargetSides.put(AAReefTarget.of(6, true), AutoAlignStates.L6);
		hexagonTargetSides.put(AAReefTarget.of(6, false), AutoAlignStates.R6);
		REEF_TARGET_MAP = Map.copyOf(hexagonTargetSides);
	}

	public static final Map<Boolean, AutoAlignStates> SOURCE_TARGET_MAP = Map.of(true, AutoAlignStates.LEFT_SOURCE, false, AutoAlignStates.RIGHT_SOURCE);

	public static final Map<Integer, LEDStates> LED_TO_REEF_LEVEL = Map.of(1, LEDStates.L1, 2, LEDStates.L2, 3, LEDStates.L3, 4, LEDStates.L4);
}
