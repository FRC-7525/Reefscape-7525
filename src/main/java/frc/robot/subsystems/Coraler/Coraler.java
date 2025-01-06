package frc.robot.subsystems.Coraler;

import static frc.robot.subsystems.Coraler.CoralerConstants.*;

import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Coraler extends Subsystem<CoralerStates> {

	private static Coraler instance;
	private final CoralerIO io;
	private final CoralerIOInputsAutoLogged inputs = new CoralerIOInputsAutoLogged();

	private Coraler(CoralerIO io) {
		super(SUBSYSTEM_NAME, CoralerStates.IDLE);
		this.io = io;
	}

	public static Coraler getInstance() {
		if (instance == null) {
			switch (GlobalConstants.ROBOT_MODE) {
				case SIM -> instance = new Coraler(new CoralerIOSim());
				case REAL -> instance = new Coraler(new CoralerIOSparkMax());
				case REPLAY -> instance = new Coraler(new CoralerIO() {});
				default -> throw new IllegalStateException(
					"Unexpected value: " + GlobalConstants.ROBOT_MODE
				);
			}
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setVelocity(getState().getVelocity());
		Logger.processInputs(SUBSYSTEM_NAME, inputs);
	}
}
