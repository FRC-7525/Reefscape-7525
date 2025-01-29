package frc.robot.Subsystems.Coraler;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Coraler extends Subsystem<CoralerStates> {

	private static Coraler instance;
	private final CoralerIO io;
	private final CoralerIOInputsAutoLogged inputs = new CoralerIOInputsAutoLogged();

	private Coraler() {
		super(SUBSYSTEM_NAME, CoralerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new CoralerIOSim();
			case REAL -> new CoralerIOSparkMax();
			case TESTING -> new CoralerIOSparkMax();
			case REPLAY -> new CoralerIO() {};
		};
	}

	public static Coraler getInstance() {
		if (instance == null) {
			instance = new Coraler();
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setVelocity(getState().getVelocity());
		Logger.processInputs(SUBSYSTEM_NAME, inputs);
	}

	public boolean isEmpty() { //TODO ADD AN ACTUAL IMPLEMENTATION. THIS IS JUST FOR AUTO SIM
		return !(getStateTime() > 0.5);
	}
}
