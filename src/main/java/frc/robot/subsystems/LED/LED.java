package frc.robot.Subsystems.LED;

import static frc.robot.Subsystems.LED.LEDConstants.SUBSYSTEM_NAME;

import frc.robot.GlobalConstants;
import frc.robot.Subsystems.LED.LEDIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class LED extends Subsystem<LEDStates> {

	private static LED instance;
	private LEDIO io;
	private LEDIOInputsAutoLogged inputs;

	private LED() {
		super(SUBSYSTEM_NAME, LEDStates.DISABLED);
		io = switch (GlobalConstants.ROBOT_MODE) {
			case SIM -> new LEDIOSim();
			case REAL -> new LEDIOReal();
			default -> new LEDIO() {};
		};

		inputs = new LEDIOInputsAutoLogged();
	}

	public static LED getInstance() {
		if (instance == null) {
			instance = new LED();
		}
		return instance;
	}

	@Override
	public void runState() {
		io.setPWMState(getState().getSignal());
		Logger.processInputs(SUBSYSTEM_NAME, inputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
	}
}
