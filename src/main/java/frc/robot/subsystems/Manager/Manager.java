package frc.robot.Subsystems.Manager;

import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Subsystems.Manager.ManagerConstants.*;

import frc.robot.Subsystems.Algaer.Algaer;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Manager extends Subsystem<ManagerStates> {

	private static Manager instance;

	private final Drive drive = Drive.getInstance();
	private final Elevator elevator = Elevator.getInstance();
	private final Coraler coraler = Coraler.getInstance();
	private final Algaer algaer = Algaer.getInstance();
	private final AutoAlign autoAlign = AutoAlign.getInstance();

	public boolean leftSourceSelected = false;
	// The runnable triggers and regular triggers will race eachother and if runnable loses our code crashes because you can't map -1 to a map without -1 :boiled: (fix it to change -1 to 1, im not gona do that tho)
	public int driverReefScoringLevel = -1;
	public int operatorReefScoringLevel = -1;
	public int hexagonTargetSide = -1;
	public boolean scoringReefLeft = false;

	private Manager() {
		super("Manager", ManagerStates.IDLE);
		// Toggling which source to AA to
		addRunnableTrigger(
			() -> this.leftSourceSelected = true,
			DRIVER_CONTROLLER::getLeftBumperButtonPressed
		);
		addRunnableTrigger(
			() -> this.leftSourceSelected = false,
			DRIVER_CONTROLLER::getRightBumperButtonPressed
		);

		// Toggling which level to score at (manual)
		addRunnableTrigger(
			() -> this.driverReefScoringLevel = 1,
			() -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD
		);
		addRunnableTrigger(
			() -> this.driverReefScoringLevel = 2,
			() -> DRIVER_CONTROLLER.getPOV() == RIGHT_DPAD
		);
		addRunnableTrigger(
			() -> this.driverReefScoringLevel = 3,
			() -> DRIVER_CONTROLLER.getPOV() == LEFT_DPAD
		);
		addRunnableTrigger(
			() -> this.driverReefScoringLevel = 4,
			() -> DRIVER_CONTROLLER.getPOV() == UP_DPAD
		);

		// Toggling which level to score at (auto align)
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 1, () -> FIGHT_STICK_1.getRawButtonPressed(1));
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 2, () -> FIGHT_STICK_1.getRawButtonPressed(2));
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 3, () -> FIGHT_STICK_1.getRawButtonPressed(3));
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 4, () -> FIGHT_STICK_1.getRawButtonPressed(4));

		// Togling which side of the hexagon to score at (auto align)
		addRunnableTrigger(() -> this.hexagonTargetSide = 1, () -> FIGHT_STICK_2.getRawButtonPressed(2));
		addRunnableTrigger(() -> this.hexagonTargetSide = 2, () -> FIGHT_STICK_2.getRawButtonPressed(3));
		addRunnableTrigger(() -> this.hexagonTargetSide = 3, () -> FIGHT_STICK_2.getRawButtonPressed(4));
		addRunnableTrigger(() -> this.hexagonTargetSide = 4, () -> FIGHT_STICK_2.getRawButtonPressed(5));
		addRunnableTrigger(() -> this.hexagonTargetSide = 5, () -> FIGHT_STICK_2.getRawButtonPressed(6));
		addRunnableTrigger(() -> this.hexagonTargetSide = 6, () -> FIGHT_STICK_2.getRawButtonPressed(7));

		// Toggling Left or Right Hexagon Side Scoring (auto align)
		addRunnableTrigger(() -> this.scoringReefLeft = true, () -> FIGHT_STICK_1.getRawButtonPressed(5));
		addRunnableTrigger(() -> this.scoringReefLeft = false, () -> FIGHT_STICK_1.getRawButtonPressed(6));

		// Climbing
		// TODO: Check with yussuf if holding down the trigger for letting up the climber then releasing is good
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.CLIMBING,
			() -> DRIVER_CONTROLLER.getLeftTriggerAxis() > 0.5
		);
		addTrigger(
			ManagerStates.CLIMBING,
			ManagerStates.IDLE,
			() -> DRIVER_CONTROLLER.getLeftTriggerAxis() == 0
		);

		// Intaking at Coral Station
		// TODO: Cache left or right bumper presses in a variable for AA to the correct source
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.INTAKING_CORALER,
			() ->
				DRIVER_CONTROLLER.getLeftBumperButtonPressed() ||
				DRIVER_CONTROLLER.getRightBumperButtonPressed()
		);
		addTrigger(
			ManagerStates.INTAKING_CORALER,
			ManagerStates.IDLE,
			() ->
				DRIVER_CONTROLLER.getLeftBumperButtonReleased() &&
				DRIVER_CONTROLLER.getRightBumperButtonReleased()
		);

		// Intaking Algae
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.INTAKING_ALGAE_LOW,
			DRIVER_CONTROLLER::getBButtonPressed
		);
		addTrigger(
			ManagerStates.INTAKING_ALGAE_HIGH,
			ManagerStates.INTAKING_ALGAE_LOW,
			() -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD
		);
		addTrigger(
			ManagerStates.INTAKING_ALGAE_LOW,
			ManagerStates.INTAKING_ALGAE_HIGH,
			() -> DRIVER_CONTROLLER.getPOV() == UP_DPAD
		);
		addTrigger(
			ManagerStates.INTAKING_ALGAE_HIGH,
			ManagerStates.IDLE,
			DRIVER_CONTROLLER::getBButtonPressed
		);
		addTrigger(
			ManagerStates.INTAKING_ALGAE_LOW,
			ManagerStates.IDLE,
			DRIVER_CONTROLLER::getBButtonPressed
		);

		// Scoring Algae at Processor
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.GOING_PROCESSOR,
			DRIVER_CONTROLLER::getAButtonPressed
		);
		addTrigger(
			ManagerStates.GOING_PROCESSOR,
			ManagerStates.SCORING_PROCESSOR,
			() -> elevator.nearTarget() && algaer.nearTarget()
		);
		addTrigger(
			ManagerStates.SCORING_PROCESSOR,
			ManagerStates.IDLE,
			DRIVER_CONTROLLER::getAButtonPressed
		);

		// Scoring Reef Manual
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.TRANSITIONING_SCORING_REEF,
			() -> DRIVER_CONTROLLER.getPOV() != -1
		);
		addTrigger(
			ManagerStates.TRANSITIONING_SCORING_REEF,
			ManagerStates.SCORING_REEF_MANUAL,
			DRIVER_CONTROLLER::getYButtonPressed
		);
		addTrigger(
			ManagerStates.SCORING_REEF_MANUAL,
			ManagerStates.IDLE,
			DRIVER_CONTROLLER::getYButtonPressed
		);

		// Scoring Reef Auto Align
		// TODO: Implement operator input & make near target scale to setpoint
		addTrigger(ManagerStates.IDLE, ManagerStates.AUTO_ALIGN_FAR, () -> FIGHT_STICK_2.getRawButtonPressed(8));
		addTrigger(
			ManagerStates.AUTO_ALIGN_FAR,
			ManagerStates.AUTO_ALIGN_CLOSE,
			autoAlign::readyForClose
		);
		addTrigger(
			ManagerStates.AUTO_ALIGN_CLOSE,
			ManagerStates.SCORING_REEF_AA,
			autoAlign::nearTarget
		);
		addTrigger(
			ManagerStates.SCORING_REEF_AA,
			ManagerStates.IDLE,
			DRIVER_CONTROLLER::getYButtonPressed
		);
	}

	public static Manager getInstance() {
		if (instance == null) {
			instance = new Manager();
		}
		return instance;
	}

	public boolean isLeftSourceSelected() {
		return leftSourceSelected;
	}

	public int getDriverReefScoringLevel() {
		return driverReefScoringLevel;
	}

	public int getOperatorReefScoringLevel() {
		return operatorReefScoringLevel;
	}

	public int getHexagonTargetSide() {
		return hexagonTargetSide;
	}

	public boolean isScoringReefLeft() {
		return scoringReefLeft;
	}

	@Override
	public void runState() {
		Logger.recordOutput(ManagerConstants.SUBSYSTEM_NAME + "/State Time", getStateTime());

		// Set States
		elevator.setState(getState().getElevatorState());
		coraler.setState(getState().getCoralerState());

		// Periodics
		autoAlign.periodic();
		drive.periodic();
		elevator.periodic();
		coraler.periodic();

		// STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!
		if (DRIVER_CONTROLLER.getXButtonPressed() || FIGHT_STICK_2.getRawButtonPressed(1)) {
			setState(ManagerStates.IDLE);
		}
	}
}
