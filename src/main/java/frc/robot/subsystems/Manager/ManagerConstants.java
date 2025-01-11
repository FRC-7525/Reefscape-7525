package frc.robot.Subsystems.Manager;


import java.util.HashMap;
import java.util.Map;

import frc.robot.Subsystems.Elevator.ElevatorStates;

public final class ManagerConstants {

	public static final String SUBSYSTEM_NAME = "Manager";

	public static final double DOWN_DPAD = 180;
	public static final double UP_DPAD = 0;
	public static final double LEFT_DPAD = 270;
	public static final double RIGHT_DPAD = 90;

	public static final Map<Integer, ElevatorStates> REEF_SCORING_LEVELS = Map.of(
		1, ElevatorStates.L1,
		2, ElevatorStates.L2,
		3, ElevatorStates.L3,
		4, ElevatorStates.L4
	);
}
