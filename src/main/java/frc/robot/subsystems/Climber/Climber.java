package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Climber.ClimberConstants.SUBSYSTEM_NAME;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
		io.setSetpoint(getState().getTargetHeight());
		io.updateInputs(inputs);

		Logger.processInputs(ClimberConstants.SUBSYSTEM_NAME, inputs);
		Logger.recordOutput("Climber/Setpoint", new Pose3d(new Translation3d(0, 0, getState().getTargetHeight().in(Meters)), new Rotation3d()));
		Logger.recordOutput("Climber/Position", new Pose3d(new Translation3d(0, 0, io.getPosition().in(Meters)), new Rotation3d()));
	}

	public void stop() {
		io.stop();
	}

	public boolean nearSetpoint() {
		return io.nearSetpoint();
	}
}
