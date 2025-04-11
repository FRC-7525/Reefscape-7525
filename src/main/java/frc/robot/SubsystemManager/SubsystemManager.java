package frc.robot.SubsystemManager;

import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.SubsystemManager.SubsystemManagerConstants.*;
import static frc.robot.SubsystemManager.SubsystemManagerStates.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.Subsystems.AutoAlign.AATypeManager.AATypeManager;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.Passthrough.Passthrough;
import frc.robot.Subsystems.Vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.team7525.misc.Tracer;
import org.team7525.subsystem.Subsystem;

public class SubsystemManager extends Subsystem<SubsystemManagerStates> {

	private static SubsystemManager instance = new SubsystemManager();

	public Boolean leftSourceSelected = false;

	public int driverReefScoringLevel = 3;
	public int operatorReefScoringLevel = 1;
	public int hexagonTargetSide = 1;
	public boolean scoringReefLeft = false;
	private final Debouncer bouncing = new Debouncer(0.05, DebounceType.kBoth);
	private final Debouncer L1Debouncer = new Debouncer(L1_DEBOUNCE_TIME, DebounceType.kBoth);
 
	private SubsystemManager() {
		super(SUBSYSTEM_NAME, SubsystemManagerStates.IDLE);
		// Toggling which level to score at (manual)
		addRunnableTrigger(() -> this.driverReefScoringLevel = 1, () -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD);
		addRunnableTrigger(() -> this.driverReefScoringLevel = 2, () -> DRIVER_CONTROLLER.getPOV() == RIGHT_DPAD);
		addRunnableTrigger(() -> this.driverReefScoringLevel = 3, () -> DRIVER_CONTROLLER.getPOV() == LEFT_DPAD);
		addRunnableTrigger(() -> this.driverReefScoringLevel = 4, () -> DRIVER_CONTROLLER.getPOV() == UP_DPAD);

		// // Toggling which level to score at (auto align)
		addRunnableTrigger(
			() -> {
				this.operatorReefScoringLevel = 1;
				this.driverReefScoringLevel = 1;
			},
			() -> OPERATOR_CONTROLLER.getRawButtonPressed(2)
		);
		addRunnableTrigger(
			() -> {
				this.operatorReefScoringLevel = 2;
				this.driverReefScoringLevel = 2;
			},
			() -> OPERATOR_CONTROLLER.getRawButtonPressed(6)
		);
		addRunnableTrigger(
			() -> {
				this.operatorReefScoringLevel = 3;
				this.driverReefScoringLevel = 3;
			},
			() -> OPERATOR_CONTROLLER.getRawAxis(0) < -AXIS_RECOGNITION_POINT
		);
		addRunnableTrigger(
			() -> {
				this.operatorReefScoringLevel = 4;
				this.driverReefScoringLevel = 4;
			},
			() -> OPERATOR_CONTROLLER.getRawAxis(0) > AXIS_RECOGNITION_POINT
		);

		// Togling which side of the hexagon to score at (auto align)
		addRunnableTrigger(() -> this.hexagonTargetSide = 1, () -> OPERATOR_CONTROLLER.getRawButtonPressed(8));
		addRunnableTrigger(() -> this.hexagonTargetSide = 2, () -> OPERATOR_CONTROLLER.getRawButtonPressed(9));
		addRunnableTrigger(() -> this.hexagonTargetSide = 3, () -> OPERATOR_CONTROLLER.getRawButtonPressed(10));
		addRunnableTrigger(() -> this.hexagonTargetSide = 4, () -> OPERATOR_CONTROLLER.getRawButtonPressed(11));
		addRunnableTrigger(() -> this.hexagonTargetSide = 5, () -> OPERATOR_CONTROLLER.getRawButtonPressed(12));
		addRunnableTrigger(() -> this.hexagonTargetSide = 6, () -> OPERATOR_CONTROLLER.getRawButtonPressed(7));

		// Toggling Left or Right Hexagon Side Scoring (auto align)
		addRunnableTrigger(() -> this.scoringReefLeft = true, () -> OPERATOR_CONTROLLER.getRawButton(1)); // 1
		addRunnableTrigger(() -> this.scoringReefLeft = false, () -> OPERATOR_CONTROLLER.getRawButton(3)); // 3

		// Intaking at Coral Station with AA
		addTrigger(IDLE, INTAKING_CORALER, () -> {
			// Is either bumper pressed, if so transition. Configure which source to align to based on which is pressed
			if (DRIVER_CONTROLLER.getLeftBumperButtonPressed()) {
				this.leftSourceSelected = true;
				return true;
			} else if (DRIVER_CONTROLLER.getRightBumperButtonPressed()) {
				this.leftSourceSelected = false;
				return true;
			} else {
				return false;
			}
		});
		addTrigger(INTAKING_CORALER, INTAKING_CORALER_AA_OFF, () -> AutoAlign.getInstance().nearGoalSource());
		addTrigger(INTAKING_CORALER, IDLE, () -> bouncing.calculate(Coraler.getInstance().hasGamepiece()));
		addTrigger(IDLE, OUTTAKING, () -> DRIVER_CONTROLLER.getLeftTriggerAxis() > 0.8);

		// Manual
		addTrigger(IDLE, INTAKING_CORALER_AA_OFF, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(INTAKING_CORALER_AA_OFF, IDLE, () -> DRIVER_CONTROLLER.getXButtonPressed() || bouncing.calculate(Coraler.getInstance().hasGamepiece()));

		// Scoring Reef Manual
		addTrigger(IDLE, TRANSITIONING_SCORING_REEF, () -> DRIVER_CONTROLLER.getPOV() != -1);
		addTrigger(TRANSITIONING_SCORING_REEF, SCORING_REEF_MANUAL, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(SCORING_REEF_MANUAL, IDLE, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(SCORING_L1, IDLE, DRIVER_CONTROLLER::getYButtonPressed);

		addTrigger(SCORING_REEF_MANUAL, SCORING_L1, () -> driverReefScoringLevel == 1 && L1Debouncer.calculate(!Coraler.getInstance().hasGamepiece()));

		// Auto ONLY transition for alignment
		addTrigger(SCORING_REEF_MANUAL, IDLE, () -> DriverStation.isAutonomous() && getStateTime() > SCORING_TIME);
		addTrigger(TRANSITIONING_SCORING_REEF, SCORING_REEF_MANUAL, () -> DriverStation.isAutonomous() && Elevator.getInstance().nearTarget());
		addTrigger(AUTO_ALIGN_CLOSE, TRANSITIONING_SCORING_REEF, () -> {
			return AutoAlign.getInstance().timedOut() && DriverStation.isAutonomous();
		});

		// Scoring Reef Auto Align
		addTrigger(IDLE, AUTO_ALIGN_FAR, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(AUTO_ALIGN_FAR, AUTO_ALIGN_CLOSE, AutoAlign.getInstance()::readyForClose);
		addTrigger(AUTO_ALIGN_CLOSE, TRANSITIONING_SCORING_REEF, AutoAlign.getInstance()::nearGoal);

		// Elevator Zeroing
		addTrigger(IDLE, ZEROING_ELEVATOR, () -> OPERATOR_CONTROLLER.getRawButtonPressed(4));
		addTrigger(ZEROING_ELEVATOR, IDLE, () -> OPERATOR_CONTROLLER.getRawButtonPressed(4));
	}

	public static SubsystemManager getInstance() {
		if (instance == null) {
			instance = new SubsystemManager();
		}
		return instance;
	}

	public boolean getLeftSourceSelected() {
		return leftSourceSelected;
	}

	public boolean getScoringReefLeft() {
		return scoringReefLeft;
	}

	public int getHexagonTargetSide() {
		return hexagonTargetSide;
	}

	public int getDriverReefScoringLevel() {
		return driverReefScoringLevel;
	}

	public int getOperatorReefScoringLevel() {
		return operatorReefScoringLevel;
	}

	// For use by auto manager & auto commands
	public void setDriverReefScoringLevel(int driverReefScoringLevel) {
		this.driverReefScoringLevel = driverReefScoringLevel;
	}

	public void setOperatorScoringLevel(int operatorReefScoringLevel) {
		this.operatorReefScoringLevel = operatorReefScoringLevel;
	}

	public void setLeftSourceTargeted(boolean leftSourceTargeted) {
		this.leftSourceSelected = leftSourceTargeted;
	}

	public void setHexagonTargetSide(int targetSide) {
		this.hexagonTargetSide = targetSide;
	}

	public void setScoringReefLeft(boolean scoringReefLeft) {
		this.scoringReefLeft = scoringReefLeft;
	}

	public double getTime() {
		return getStateTime();
	}

	@Override
	public void runState() {
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/State Time", getStateTime());
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/State String", getState().getStateString());
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Selected Reef Side", hexagonTargetSide);
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Selected Reef Level", operatorReefScoringLevel);
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Left Pose Selected", scoringReefLeft);
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Driver Reef Level", driverReefScoringLevel);
		if (Robot.isRedAlliance) {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "RED");
		} else {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "BLUE");
		}

		// Set States, drive and vision are rogue so you don't need to set state
		Elevator.getInstance().setState(getState().getElevatorStateSupplier().get());
		Coraler.getInstance().setState(getState().getCoralerState());
		AutoAlign.getInstance().setState(getState().getAutoAlignSupplier().get());
		LED.getInstance().setState(getState().getLedStateSupplier().get());
		Passthrough.getInstance().setState(getState().getPassthroughState());

		// Periodics
		Tracer.traceFunc("AutoAlignPeriodic", AutoAlign.getInstance()::periodic);
		Tracer.traceFunc("ElevatorPeriodic", Elevator.getInstance()::periodic);
		Tracer.traceFunc("CoralerPeriodic", Coraler.getInstance()::periodic);
		Tracer.traceFunc("VisionPeriodic", Vision.getInstance()::periodic);
		Tracer.traceFunc("DrivePeriodic", Drive.getInstance()::periodic);
		Tracer.traceFunc("AATypeManagerPeriodic", AATypeManager.getInstance()::periodic);
		Tracer.traceFunc("LEDPeriodic", LED.getInstance()::periodic);
		Tracer.traceFunc("PassthroughPeriodic", Passthrough.getInstance()::periodic);

		// STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!
		if (DRIVER_CONTROLLER.getBackButtonPressed() || OPERATOR_CONTROLLER.getRawButtonPressed(5)) {
			setState(SubsystemManagerStates.IDLE);
		}
	}

	/*
	 * This essentially ensures you don't automatically
	 * transition out of the next state you're "pressed"
	 * isn't checked in the previous state so you can "pre"-transition
	 * a state before you're even there which is mad annoying.
	 * The real solution for this is adding something to the subsystem class so this
	 * is ONLY called when a state is actually transitioned
	 * (attempted but uh it wasn't working so we cooked up this temp fix cause its not
	 * that intesive)
	 */
	public void clearButtonPressCache() {
		DRIVER_CONTROLLER.getAButtonPressed();
		DRIVER_CONTROLLER.getBButtonPressed();
		DRIVER_CONTROLLER.getXButtonPressed();
		DRIVER_CONTROLLER.getYButtonPressed();
		DRIVER_CONTROLLER.getBackButtonPressed();
		DRIVER_CONTROLLER.getStartButtonPressed();
		DRIVER_CONTROLLER.getLeftBumperButtonPressed();
		DRIVER_CONTROLLER.getRightBumperButtonPressed();
		DRIVER_CONTROLLER.getLeftStickButtonPressed();
		DRIVER_CONTROLLER.getRightStickButtonPressed();

		for (int i = 1; i < 13; i++) {
			OPERATOR_CONTROLLER.getRawButtonPressed(i);
		}
	}

	@Override
	public void periodic() {
		super.periodic();
		clearButtonPressCache();
	}

	@Override
	protected void stateExit() {}
}
