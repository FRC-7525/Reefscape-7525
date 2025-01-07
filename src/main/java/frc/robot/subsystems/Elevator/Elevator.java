package frc.robot.Subsystems.Elevator;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.SUBSYSTEM_NAME;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Elevator extends Subsystem<ElevatorStates> {

	private static Elevator instance;

	private ElevatorIO io;
	private ElevatorIOInputsAutoLogged inputs;

	private Elevator() {
		super(SUBSYSTEM_NAME, ElevatorStates.DOWN);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new ElevatorIOSim();
			case REAL -> new ElevatorIOReal();
			case TESTING -> new ElevatorIOReal();
			case REPLAY -> new ElevatorIOSim();
		};
		inputs = new ElevatorIOInputsAutoLogged();
	}

	public static Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}
		return instance;
	}

	@Override
	protected void runState() {
		if (!io.isZeroed()) {
			io.zero();
		} else {
			io.setHeightGoalpoint(getState().getTargetHeight());
			io.runElevator();
		}

		io.updateInputs(inputs);
		Logger.processInputs(ElevatorConstants.SUBSYSTEM_NAME, inputs);
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}
}
