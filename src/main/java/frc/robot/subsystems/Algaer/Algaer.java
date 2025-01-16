package frc.robot.Subsystems.Algaer;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Algaer extends Subsystem<AlgaerStates> {

	private AlgaerIO io;
	private AlgaerIOInputsAutoLogged inputs;
	private static Algaer instance;

	private Algaer() {
		super("Algaer", AlgaerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new AlgaerIOSim();
			case REAL -> new AlgaerIOReal();
			case TESTING -> new AlgaerIOReal();
			case REPLAY -> new AlgaerIOSim();
		};
		inputs = new AlgaerIOInputsAutoLogged();
	}

	@Override
	protected void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeedSetpoint());

		io.updateInputs(inputs);
		Logger.processInputs("Algaer", inputs);
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}

	public static Algaer getInstance() {
		if (instance == null) {
			instance = new Algaer();
		}
		return instance;
	}
}
