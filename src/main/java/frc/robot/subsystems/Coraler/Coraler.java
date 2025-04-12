package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

		//TODO: DELETE WHEN DONE TUNING
		SmartDashboard.putNumber("L1 Scoring Velocity", CORALING_VELOCITY_L1_SCORING);
		SmartDashboard.putNumber("L1 Velocity", CORALING_VELOCITY_L1);
	}

	public static Coraler getInstance() {
		if (instance == null) {
			instance = new Coraler();
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setVelocity(getState().getVelocitySupplier().get());
		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);

		Logger.recordOutput(SUBSYSTEM_NAME + "/Stator Current", io.getMotor().getStatorCurrent().getValueAsDouble());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Has Gamepiece", this.hasGamepiece());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Current Sensed?", this.currentSenseGamepiece());
	}

	public boolean currentSenseGamepiece() {
		return debouncer.calculate(io.currentLimitReached());
	}

	public boolean gamepieceLeft() {
		return io.gamepieceLeft();
	}

	public boolean hasGamepiece() {
		return io.hasGamepiece();
	}

	public TalonFX getMotor() {
		return io.getMotor();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}
