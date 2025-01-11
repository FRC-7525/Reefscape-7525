package frc.robot.subsystems.Algaer;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Algaer extends Subsystem<AlgaerStates> {

	AlgaerIO io;
	AlgaerIOInputsAutoLogged inputs;
	private static Algaer instance;

	private Algaer(AlgaerIO io) {
		super("Algaer", AlgaerStates.IDLE);
		this.io = io;
		inputs = new AlgaerIOInputsAutoLogged();
	}

	@Override
	protected void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeedSetpoint());

		io.updateInputs(inputs);
		Logger.processInputs("Algaer", inputs);
	}

	public static Algaer getInstance() {
		if (instance == null) {
			AlgaerIO AlgaerIO =
				switch (ROBOT_MODE) {
					case SIM -> new AlgaerIOSim();
					case REAL -> new AlgaerIOReal();
					case TESTING -> new AlgaerIOReal();
					case REPLAY -> new AlgaerIOSim();
				};
			instance = new Algaer(AlgaerIO);
		}
		return instance;
	}
}
