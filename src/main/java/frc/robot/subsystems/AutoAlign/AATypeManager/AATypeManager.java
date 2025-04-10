package frc.robot.Subsystems.AutoAlign.AATypeManager;

import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.AutoAlign.AutoAlignStates;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AATypeManager extends Subsystem<AATypeManagerStates> {

	private static AATypeManager instance;

	private AATypeManager() {
		super("AA Type Manager", AATypeManagerStates.OFF);
		// Start AA
		addTrigger(AATypeManagerStates.OFF, AATypeManagerStates.REPULSOR, () -> AutoAlign.getInstance().getState() != AutoAlignStates.OFF);

		// Repulsor to regular -> finished
		addTrigger(AATypeManagerStates.REPULSOR, AATypeManagerStates.REGULAR, () -> AutoAlign.getInstance().closeEnoughToIgnore() || !AutoAlign.getInstance().willCollideWithReef());
		addTrigger(AATypeManagerStates.REGULAR, AATypeManagerStates.OFF, () -> AutoAlign.getInstance().nearGoal());

		// Stop AA incase AA stops for some reason
		addTrigger(AATypeManagerStates.REGULAR, AATypeManagerStates.OFF, () -> AutoAlign.getInstance().getState() == AutoAlignStates.OFF);
		addTrigger(AATypeManagerStates.REPULSOR, AATypeManagerStates.OFF, () -> AutoAlign.getInstance().getState() == AutoAlignStates.OFF);
	}

	public static AATypeManager getInstance() {
		if (instance == null) {
			instance = new AATypeManager();
		}
		return instance;
	}

	@Override
	public void runState() {
		Logger.recordOutput("AA Type Manager/State", getState().getStateString());
		Logger.recordOutput("AutoAlign/UsingRepulsor", getState() == AATypeManagerStates.REPULSOR);
		getState().runAlignmentRunnable();
	}

	@Override
	protected void stateExit() {
		if (getState() == AATypeManagerStates.REPULSOR) {
			AutoAlign.getInstance().transitionToRegular();
		}
	}
}
