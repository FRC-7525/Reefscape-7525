package frc.robot.Subsystems.LED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class LED extends Subsystem<LEDStates> {

	private static LED instance;
	private LEDIO io;
	private LEDIOInputsAutoLogged inputs;

	// private LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

	private LED() {
		super("LEDs", LEDStates.RED);
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
		Logger.processInputs("LED", inputs);
		SmartDashboard.putString("LED State", getState().getStateString());
	}
}
