package frc.robot.AutoManager;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.AutoManager.AutoConstants.CLOSE_DISTANCE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemManager.SubsystemManager;
import frc.robot.SubsystemManager.SubsystemManagerStates;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;

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

		private final SubsystemManager manager = SubsystemManager.getInstance();

		public static IntakeCoral getCoral() {
			return AutoCommands.getInstance().new IntakeCoral();
		}

		@Override
		public void initialize() {
			manager.setState(SubsystemManagerStates.INTAKING_CORALER_AA_OFF);
		}

		@Override
		public boolean isFinished() {
			return manager.getState() != SubsystemManagerStates.INTAKING_CORALER_AA_OFF;
		}
	}

	public class ScoreReef extends Command {

		private final int scoringLevel;

		private ScoreReef(int scoringLevel) {
			this.scoringLevel = scoringLevel;
		}

		public static ScoreReef atLevel(int scoringLevel) {
			return AutoCommands.getInstance().new ScoreReef(scoringLevel);
		}

		@Override
		public void initialize() {
			SubsystemManager.getInstance().setDriverReefScoringLevel(scoringLevel);
			SubsystemManager.getInstance().setState(SubsystemManagerStates.SCORING_REEF_MANUAL);
		}

		@Override
		public boolean isFinished() {
			return SubsystemManager.getInstance().getState() == SubsystemManagerStates.IDLE;
		}
	}

	public class ProcessAlgae extends Command {}

	public class GoToElevatorLevel extends Command {

		private final int scoringLevel;

		private GoToElevatorLevel(int scoringLevel) {
			this.scoringLevel = scoringLevel;
		}

		public static GoToElevatorLevel atLevel(int scoringLevel) {
			return AutoCommands.getInstance().new GoToElevatorLevel(scoringLevel);
		}

		@Override
		public void initialize() {
			SubsystemManager.getInstance().setDriverReefScoringLevel(scoringLevel);
			SubsystemManager.getInstance().setState(SubsystemManagerStates.TRANSITIONING_SCORING_REEF);
		}

		@Override
		public boolean isFinished() {
			return Elevator.getInstance().nearTarget();
		}
	}

	public class ReturnToIdle extends Command {

		private final SubsystemManager manager = SubsystemManager.getInstance();

		public static ReturnToIdle get() {
			return AutoCommands.getInstance().new ReturnToIdle();
		}

		@Override
		public void initialize() {
			manager.setState(SubsystemManagerStates.IDLE);
		}

		@Override
		public boolean isFinished() {
			return manager.getState() == SubsystemManagerStates.IDLE;
		}
	}

	public class Transition extends Command {

		private final SubsystemManager manager = SubsystemManager.getInstance();
		private final Drive drive = Drive.getInstance();

		public static Transition get() {
			return AutoCommands.getInstance().new Transition();
		}

		@Override
		public void initialize() {
			manager.setState(SubsystemManagerStates.IDLE);
		}

		@Override
		public void execute() {
			System.out.println("hi");
			manager.setState(SubsystemManagerStates.IDLE);
		}

		@Override
		public boolean isFinished() {
			return drive.getPose().getTranslation().getDistance(AutoManager.getInstance().getEndPose().getTranslation()) < CLOSE_DISTANCE.in(Meters);
		}
	}
}
