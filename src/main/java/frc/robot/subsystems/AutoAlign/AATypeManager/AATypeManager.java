package frc.robot.Subsystems.AutoAlign.AATypeManager;

import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import org.team7525.subsystem.Subsystem;

public class AATypeManager extends Subsystem<AATypeManagerStates> {

	private static AATypeManager instance;

	private AATypeManager() {
		super("AA Type Manager", AATypeManagerStates.OFF);
		// Start AA
		addTrigger(AATypeManagerStates.OFF, AATypeManagerStates.REPULSOR, () -> AutoAlign.getInstance().getState() != AutoAlignStates.OFF);

		addTrigger(AATypeManagerStates.REPULSOR, AATypeManagerStates.REGULAR, () -> AutoAlign.getInstance().closeEnoughToIgnore() || !AutoAlign.getInstance().willCollideWithReef());
		addTrigger(AATypeManagerStates.REGULAR, AATypeManagerStates.OFF, () -> AutoAlign.getInstance().getState() == AutoAlignStates.OFF);
	}

	public static AATypeManager getInstance() {
		if (instance == null) {
			instance = new AATypeManager();
		}
		return instance;
	}

	@Override
	public void runState() {
		getState().runAlignmentRunnable();
	}
}
