package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Coraler extends Subsystem<CoralerStates> {

	private static Coraler instance;
	private final CoralerIO io;
	private final CoralerIOInputsAutoLogged inputs = new CoralerIOInputsAutoLogged();
	private final Debouncer debouncer;

	private Coraler() {
		super(SUBSYSTEM_NAME, CoralerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new CoralerIOSim();
			case REAL -> new CoralerIOTalonFX();
			case TESTING -> new CoralerIOTalonFX();
		};

		debouncer = new Debouncer(DEBOUNCE_TIME.in(Seconds), DebounceType.kRising);
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
		Logger.recordOutput(SUBSYSTEM_NAME + "/Stator Current", io.getMotor().getStatorCurrent().getValueAsDouble());
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

	public boolean currentSenseGamepiece() {
		return debouncer.calculate(io.currentLimitReached());
	}

	public TalonFX getMotor() {
		return io.getMotor();
	}
}
