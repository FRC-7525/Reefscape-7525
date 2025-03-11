package frc.robot.AutoManager;

import static frc.robot.AutoManager.AutoStates.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SubsystemManager.SubsystemManager;
import frc.robot.SubsystemManager.SubsystemManagerStates;
import frc.robot.Subsystems.Elevator.Elevator;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AutoManager extends Subsystem<AutoStates> {

	public static AutoManager instance;

	private final SendableChooser<AutoScoringLocation[]> scoringLocationChooser = new SendableChooser<>();
	private final SendableChooser<Boolean> intakingLocationChooser = new SendableChooser<>();
	private final SendableChooser<Integer> scoringLevelChooser = new SendableChooser<>();
	private int orderInRoutine = 0;
	private boolean setManagerStateAlready = false;
	private boolean finishedAuto = false;

	private AutoManager() {
		super("Auto", AutoStates.SCORING_CORAL);
		Logger.recordOutput("Auto/State", getState().getStateString());
		intakingLocationChooser.setDefaultOption("Right", false);
		intakingLocationChooser.addOption("Left", true);

		scoringLevelChooser.setDefaultOption("L4", 4);
		scoringLevelChooser.addOption("L3", 3);
		scoringLevelChooser.addOption("L2", 2);
		scoringLevelChooser.addOption("L1", 1);

		scoringLocationChooser.setDefaultOption(
			"I J K L A B| 6 Piece | Left",
			new AutoScoringLocation[] { new AutoScoringLocation(true, 5), new AutoScoringLocation(false, 5), new AutoScoringLocation(true, 6), new AutoScoringLocation(false, 6), new AutoScoringLocation(true, 1), new AutoScoringLocation(false, 1) }
		);
		scoringLocationChooser.addOption("J K L | 3 Piece | Left", new AutoScoringLocation[] { new AutoScoringLocation(false, 5), new AutoScoringLocation(false, 6), new AutoScoringLocation(true, 6) });
		scoringLocationChooser.addOption("E D C | 3 Piece | Right", new AutoScoringLocation[] { new AutoScoringLocation(true, 3), new AutoScoringLocation(false, 2), new AutoScoringLocation(true, 2) });
		scoringLocationChooser.addOption("D C | 2 Piece | Right", new AutoScoringLocation[] { new AutoScoringLocation(false, 2), new AutoScoringLocation(true, 2) });
		scoringLocationChooser.addOption("K L | 2 Piece | Left", new AutoScoringLocation[] { new AutoScoringLocation(false, 6), new AutoScoringLocation(true, 6) });
		scoringLocationChooser.addOption("G | 1 Piece | Move Forward", new AutoScoringLocation[] { new AutoScoringLocation(true, 4) });
		scoringLocationChooser.addOption("A | 1 Piece | Move Forward", new AutoScoringLocation[] { new AutoScoringLocation(false, 1) });

		addTrigger(SCORING_CORAL, GOING_DOWN, () -> {
			if (getStateTime() < 0.5) {
				return false;
			}
			boolean triggered = SubsystemManager.getInstance().getState() == SubsystemManagerStates.IDLE && Elevator.getInstance().nearTarget();
			if (triggered) {
				orderInRoutine += 1;
				setManagerStateAlready = false;
			}
			return triggered;
		});

		addTrigger(GOING_DOWN, INTAKING_CORAL, () -> {
			if (scoringLocationChooser.getSelected().length == 1) return false;

			boolean triggered = Elevator.getInstance().nearEnoughTarget() && getStateTime() > 0.2;
			if (triggered) {
				setManagerStateAlready = false;
			}
			return triggered;
		});

		addTrigger(INTAKING_CORAL, SCORING_CORAL, () -> {
			boolean triggered = SubsystemManager.getInstance().getState() == SubsystemManagerStates.IDLE && getStateTime() > 0.5;
			if (triggered) {
				setManagerStateAlready = false;
			}
			return triggered;
		});
		addTrigger(SCORING_CORAL, IDLE, () -> {
			boolean triggered = orderInRoutine == scoringLocationChooser.getSelected().length && getStateTime() > 0.01;
			if (triggered) {
				setManagerStateAlready = false;
				finishedAuto = true;
			}
			return triggered;
		});

		SmartDashboard.putData("Level Chooser", scoringLevelChooser);
		SmartDashboard.putData("Auto Chooser", scoringLocationChooser);
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

	public void resetAuto() {
		orderInRoutine = 0;
		finishedAuto = false;
	}

	public boolean finishedAuto() {
		return finishedAuto;
	}

	@Override
	public void runState() {
		Logger.recordOutput("Auto/State", getState().getStateString());
		Logger.recordOutput("Auto/order number", orderInRoutine);
		// Setting Manager State
		if (!setManagerStateAlready) {
			SubsystemManager.getInstance().setState(getState().getManagerState().get());
			setManagerStateAlready = true;
		}

		// Config for scoring location
		if (orderInRoutine >= scoringLocationChooser.getSelected().length) return;
		SubsystemManager.getInstance().setLeftSourceTargeted(intakingLocationChooser.getSelected());
		SubsystemManager.getInstance().setDriverReefScoringLevel(scoringLevelChooser.getSelected());
		SubsystemManager.getInstance().setOperatorScoringLevel(scoringLevelChooser.getSelected());
		SubsystemManager.getInstance().setHexagonTargetSide(scoringLocationChooser.getSelected()[orderInRoutine].hexagonTargetSide);
		SubsystemManager.getInstance().setScoringReefLeft(scoringLocationChooser.getSelected()[orderInRoutine].scoreLeftPeg);
	}

	public void setOrderInRoutine(int orderNumber) {
		this.orderInRoutine = 0;
	}

	public void setFinishedAuto(boolean finishedAuto) {
		this.finishedAuto = finishedAuto;
	}
}
