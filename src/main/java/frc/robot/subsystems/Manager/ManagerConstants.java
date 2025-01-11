package frc.robot.Subsystems.Manager;

import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import java.util.HashMap;
import java.util.Map;

public final class ManagerConstants {

	public static final String SUBSYSTEM_NAME = "Manager";

	public static final double DOWN_DPAD = 180;
	public static final double UP_DPAD = 0;
	public static final double LEFT_DPAD = 270;
	public static final double RIGHT_DPAD = 90;

	public static final Map<Integer, ElevatorStates> REEF_SCORING_LEVELS = Map.of(
		1,
		ElevatorStates.L1,
		2,
		ElevatorStates.L2,
		3,
		ElevatorStates.L3,
		4,
		ElevatorStates.L4
	);

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
	}

	public static final Map<AAReefTarget, AutoAlignStates> REEF_TARGET_MAP;
	static {
		Map<AAReefTarget, AutoAlignStates> hexagonTargetSides = new HashMap<>();
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

	public static final Map<Boolean, AutoAlignStates> SOURCE_TARGET_MAP = Map.of(
		true,
		AutoAlignStates.LEFT_SOURCE,
		false,
		AutoAlignStates.RIGHT_SOURCE
	);
}
