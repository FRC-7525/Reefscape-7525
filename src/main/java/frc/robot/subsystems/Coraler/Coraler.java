package frc.robot.Subsystems.Coraler;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

public class Coraler extends Subsystem<CoralerStates> {

	private static Coraler instance;
	private final CoralerIO io;
	private final CoralerIOInputsAutoLogged inputs = new CoralerIOInputsAutoLogged();

	private Coraler() {
		super(SUBSYSTEM_NAME, CoralerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new CoralerIOSim();
			case REAL -> new CoralerIOTalonFX();
			case TESTING -> new CoralerIOTalonFX();
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
		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);

		Logger.recordOutput(SUBSYSTEM_NAME + "/First Detector Tripped", io.firstDetectorTripped());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Second Detector Tripped", io.secondDetectorTripped());
	}

	public boolean isEmpty() {
		return !io.firstDetectorTripped() && !io.secondDetectorTripped();
	}

	public boolean justIntookGamepiece() {
		return io.firstDetectorTripped();
	}

	public boolean gamepieceCentered() {
		return io.secondDetectorTripped();
	}

	public TalonFX getMotor() {
		return io.getMotor();
	}
}
