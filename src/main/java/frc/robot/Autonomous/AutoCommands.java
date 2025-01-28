package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Subsystems.Manager.ManagerStates;

public class AutoCommands {

	public static AutoCommands instance;

	private AutoCommands() {}

	protected static AutoCommands getInstance() {
		if (instance == null) {
			instance = new AutoCommands();
		}
		return instance;
	}

	public class IntakeCoral extends Command {

		private final Manager manager = Manager.getInstance();

		public static IntakeCoral getCoral() {
			return AutoCommands.getInstance().new IntakeCoral();
		}

		@Override
		public void initialize() {
			manager.setState(ManagerStates.INTAKING_CORALER_AA_OFF);
		}

		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public class ScoreReef extends Command {

		private final Manager manager = Manager.getInstance();
		private final int scoringLevel;

		private ScoreReef(int scoringLevel) {
			this.scoringLevel = scoringLevel;
		}

		public static ScoreReef atLevel(int scoringLevel) {
			return AutoCommands.getInstance().new ScoreReef(scoringLevel);
		}

		@Override
		public void initialize() {
			manager.setDriverReefScoringLevel(scoringLevel);
			manager.setState(ManagerStates.TRANSITIONING_SCORING_REEF);
		}

		@Override
		public boolean isFinished() {
			return manager.getState() == ManagerStates.IDLE;
		}
	}

	public class ProcessAlgae extends Command {}

	public class GoToElevatorLevel extends Command {
		private final Manager manager = Manager.getInstance();
		private final int scoringLevel;

		private GoToElevatorLevel(int scoringLevel) {
			this.scoringLevel = scoringLevel;
		}

		public static GoToElevatorLevel atLevel(int scoringLevel) {
			return AutoCommands.getInstance().new GoToElevatorLevel(scoringLevel);
		}

		@Override
		public void initialize() {
			manager.setDriverReefScoringLevel(scoringLevel);
			manager.setState(ManagerStates.TRANSITIONING_SCORING_REEF);
		}

		@Override
		public boolean isFinished() {
			return manager.getState() == ManagerStates.IDLE;
		}
	}
}
