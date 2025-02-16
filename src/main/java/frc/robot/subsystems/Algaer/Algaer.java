package frc.robot.Subsystems.Algaer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Algaer.AlgaerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Algaer extends Subsystem<AlgaerStates> {

	private AlgaerIO io;
	private AlgaerIOInputsAutoLogged inputs;
	private static Algaer instance;

	private Algaer() {
		super(SUBSYSTEM_NAME, AlgaerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new AlgaerIOSim();
			case REAL -> new AlgaerIOReal();
			case TESTING -> new AlgaerIOReal();
		};
		inputs = new AlgaerIOInputsAutoLogged();
	}

	@Override
	protected void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeedSetpoint());

		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);

		// Pose of mechanism for sim!
		Logger.recordOutput("Algaer/Near Targer", io.nearTarget());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Setpoint", new Pose3d(Sim.ZEROED_TRANSLATION, new Rotation3d(Degrees.of(0), getState().getPivotSetpoint(), Degrees.of(0))));
		Logger.recordOutput(SUBSYSTEM_NAME + "/Position", new Pose3d(Sim.ZEROED_TRANSLATION, new Rotation3d(Degrees.of(0), io.getAngle(), Degrees.of(0))));
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

	public TalonFX getWheelMotor() {
		return io.getWheelMotor();
	}
}
