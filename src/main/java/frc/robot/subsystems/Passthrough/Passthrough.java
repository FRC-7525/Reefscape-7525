package frc.robot.Subsystems.Passthrough;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Passthrough.PassthroughConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import frc.robot.Subsystems.Passthrough.PassthroughIOInputsAutoLogged;

public class Passthrough extends Subsystem<PassthroughStates> {

	private static Passthrough instance;
	private final PassthroughIO io;
	private final PassthroughIOInputsAutoLogged inputs;

	private Passthrough() {
		super(SUBSYSTEM_NAME, PassthroughStates.OFF);
		inputs = new PassthroughIOInputsAutoLogged();
		this.io = switch (ROBOT_MODE) {
			case REAL -> new PassthroughIOReal();
			case SIM -> new PassthroughIOSim();
			case TESTING -> new PassthroughIOReal();
		};
	}

	public static Passthrough getInstance() {
		if (instance == null) {
			instance = new Passthrough();
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setTargetVelocity(getState().getVelocity());
		io.updateInput(inputs);

		Logger.processInputs(SUBSYSTEM_NAME, inputs);
	}
}
