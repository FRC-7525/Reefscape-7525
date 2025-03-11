package frc.robot.Subsystems.Passthrough;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Passthrough.PassthroughConstants.*;

import org.team7525.subsystem.Subsystem;

import frc.robot.Subsystems.Passthrough.PassthroughIO.PassthroughIOInputs;

public class Passthrough extends Subsystem<PassthroughStates> {

    private static Passthrough instance;
    private final PassthroughIO io;
    private final PassthroughIOInputs inputs;

    private Passthrough() {
        super(SUBSYSTEM_NAME, PassthroughStates.OFF);
        inputs = new PassthroughIOInputs();
        this.io = switch (ROBOT_MODE) {
            case REAL -> new PassthroughIOReal();
            case SIM -> new PassthroughIOSim();
            case TESTING -> new PassthroughIOReal();
        };
    }

    public static Passthrough getInstance() {
        if (instance == null) {
            instance = new Passthrough();
        }
        return instance;
    }

	@Override
	protected void runState() {
		io.setTargetVelocity(getState().getVelocity());
        io.updateInput(inputs);

	}
    
}
