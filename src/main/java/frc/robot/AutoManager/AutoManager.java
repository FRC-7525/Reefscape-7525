package frc.robot.AutoManager;

import static frc.robot.AutoManager.AutoStates.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SubsystemManager.SubsystemManager;
import frc.robot.SubsystemManager.SubsystemManagerStates;
import frc.robot.Subsystems.Elevator.Elevator;
import org.team7525.subsystem.Subsystem;

public class AutoManager extends Subsystem<AutoStates> {

	public static AutoManager instance;

	private final SendableChooser<AutoScoringLocation[]> scoringLocationChooser = new SendableChooser<>();
	private final SendableChooser<Boolean> intakingLocationChooser = new SendableChooser<>();
	private final SendableChooser<Integer> scoringLevelChooser = new SendableChooser<>();
	private int orderInRoutine = 0;
	private boolean setManagerStateAlready = false;

	private AutoManager() {
		super("Auto", AutoStates.SCORING_CORAL);
		intakingLocationChooser.setDefaultOption("Right", false);
		intakingLocationChooser.addOption("Left", true);

		scoringLevelChooser.setDefaultOption("L4", 4);
		scoringLevelChooser.addOption("L3", 3);
		scoringLevelChooser.addOption("L2", 2);
		scoringLevelChooser.addOption("L1", 1);

		scoringLocationChooser.setDefaultOption("Blue Side 6", new AutoScoringLocation[] { new AutoScoringLocation(true, 5), new AutoScoringLocation(false, 5), new AutoScoringLocation(true, 6), new AutoScoringLocation(false, 6), new AutoScoringLocation(true, 1), new AutoScoringLocation(false, 1) });
		scoringLocationChooser.addOption("Blue Side 6", new AutoScoringLocation[] { new AutoScoringLocation(true, 5), new AutoScoringLocation(false, 5), new AutoScoringLocation(true, 6), new AutoScoringLocation(false, 6), new AutoScoringLocation(true, 1), new AutoScoringLocation(false, 1) });
		scoringLocationChooser.addOption("Red Side 6", new AutoScoringLocation[] { new AutoScoringLocation(false, 3), new AutoScoringLocation(true, 3) });

		addTrigger(SCORING_CORAL, INTAKING_CORAL, () -> {
			boolean triggered = SubsystemManager.getInstance().getState() == SubsystemManagerStates.IDLE && Elevator.getInstance().nearTarget();
			if (triggered && orderInRoutine != scoringLocationChooser.getSelected().length) {
				orderInRoutine += 1;
				setManagerStateAlready = false;
			}
			return triggered;
		});
		addTrigger(INTAKING_CORAL, SCORING_CORAL, () -> SubsystemManager.getInstance().getState() == SubsystemManagerStates.IDLE);
		addTrigger(SCORING_CORAL, IDLE, () -> orderInRoutine == scoringLocationChooser.getSelected().length);

		SmartDashboard.putData("Level Chooser", scoringLevelChooser);
		SmartDashboard.putData("Side Chooser", scoringLocationChooser);
		SmartDashboard.putData("Intaking Chooser", intakingLocationChooser);
	}

	public class AutoScoringLocation {

		private boolean scoreLeftPeg;
		private int hexagonTargetSide;

		public AutoScoringLocation(boolean scoreLeftPeg, int hexagonTargetSide) {
			this.scoreLeftPeg = scoreLeftPeg;
			this.hexagonTargetSide = hexagonTargetSide;
		}

		public boolean getScoringReefLeft() {
			return scoreLeftPeg;
		}

		public int getHexagonTargetSide() {
			return hexagonTargetSide;
		}
	}

	public static AutoManager getInstance() {
		if (instance == null) {
			instance = new AutoManager();
		}
		return instance;
	}

	@Override
	public void runState() {
		// Setting Manager State
		if (!setManagerStateAlready) {
			SubsystemManager.getInstance().setState(getState().getManagerState().get());
			setManagerStateAlready = true;
		}

		// Config for scoring location
		SubsystemManager.getInstance().setLeftSourceTargeted(intakingLocationChooser.getSelected());
		SubsystemManager.getInstance().setDriverReefScoringLevel(scoringLevelChooser.getSelected());
		SubsystemManager.getInstance().setOperatorScoringLevel(scoringLevelChooser.getSelected());
		SubsystemManager.getInstance().setHexagonTargetSide(scoringLocationChooser.getSelected()[orderInRoutine].hexagonTargetSide);
		SubsystemManager.getInstance().setScoringReefLeft(scoringLocationChooser.getSelected()[orderInRoutine].scoreLeftPeg);
	}
}
