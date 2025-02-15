package frc.robot.Subsystems.Passthrough;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Passthrough.PassthroughConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Passthrough extends Subsystem<PassthroughStates> {

    private static Passthrough instance;

    private PassthroughIO passthroughIO;
    private final PassthroughIOInputsAutoLogged inputs = new PassthroughIOInputsAutoLogged();

    private Passthrough() {
        super(SUBSYSTEM_NAME, PassthroughStates.IDLE);
        passthroughIO = 
         switch (ROBOT_MODE) {
			case SIM -> new PassthroughIOSim();
			case REAL -> new PassthroughIOSparkMax();
			case TESTING -> new PassthroughIOSparkMax();
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
        passthroughIO.updateInputs(new PassthroughIO.PassthroughIOInputs());
        Logger.processInputs(SUBSYSTEM_NAME, inputs);
        passthroughIO.setVelocity(getState().getSpeedPoint());
    }

    public boolean hasGamepiece() {
        return passthroughIO.hasGamepiece() || (passthroughIO.currentLimitReached() && USE_CURRENT_SENSING);
    }
}
