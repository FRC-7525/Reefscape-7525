package frc.robot.subsystems.Climber;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.subsystems.Climber.ClimberConstants.SUBSYSTEM_NAME;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	private static Climber instance;

	private ClimberIO io;
	private ClimberIOInputsAutoLogged inputs;

	private Climber() {
		super(SUBSYSTEM_NAME, ClimberStates.DOWN);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new ClimberIOSim();
			case REAL -> new ClimberIOReal();
			case TESTING -> new ClimberIOReal();
			case REPLAY -> new ClimberIOSim();
		};
		inputs = new ClimberIOInputsAutoLogged();
	}

	public static Climber getInstance() {
		if (instance == null) {
			instance = new Climber();
		}
		return instance;
	}

	@Override
	protected void runState() {
		if (!io.isZeroed()) {
			io.zero();
		} else {
			io.setSetpoint(getState().getTargetHeight());
		}

		io.updateInputs(inputs);
		Logger.processInputs(ClimberConstants.SUBSYSTEM_NAME, inputs);
	}

	public void stop() {
		io.stop();
	}

	public boolean nearSetpoint() {
		return io.nearSetpoint();
	}
}
