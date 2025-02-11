package frc.robot.SubsystemManager;

import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.SubsystemManager.SubsystemManagerConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.Vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class SubsystemManager extends Subsystem<SubsystemManagerStates> {

	private static SubsystemManager instance = new SubsystemManager();

	private final Drive drive = Drive.getInstance();
	// private final Climber climber = Climber.getInstance();
	private final Elevator elevator = Elevator.getInstance();
	private final Coraler coraler = Coraler.getInstance();
	// private final Algaer algaer = Algaer.getInstance();
	private final AutoAlign autoAlign = AutoAlign.getInstance();
	private final Vision vision = Vision.getInstance();
	private final LED ledSubsystem = LED.getInstance();

	public Boolean leftSourceSelected = false;

	public int driverReefScoringLevel = 3;
	public int operatorReefScoringLevel = 1;
	public int hexagonTargetSide = 1;
	public boolean scoringReefLeft = false;

	private SubsystemManager() {
		super("Manager", SubsystemManagerStates.IDLE);
		// Un zero elevators (as in set the boolean to false for zeroed)
		addRunnableTrigger(elevator::resetMotorsZeroed, () -> DRIVER_CONTROLLER.getBackButtonPressed() && getState() == SubsystemManagerStates.IDLE);

		// Toggling which source to AA to
		addRunnableTrigger(() -> this.leftSourceSelected = true, DRIVER_CONTROLLER::getLeftBumperButtonPressed);
		addRunnableTrigger(() -> this.leftSourceSelected = false, DRIVER_CONTROLLER::getRightBumperButtonPressed);

		// Toggling which level to score at (manual)
		addRunnableTrigger(() -> this.driverReefScoringLevel = 1, () -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD);
		addRunnableTrigger(() -> this.driverReefScoringLevel = 2, () -> DRIVER_CONTROLLER.getPOV() == RIGHT_DPAD);
		addRunnableTrigger(() -> this.driverReefScoringLevel = 3, () -> DRIVER_CONTROLLER.getPOV() == LEFT_DPAD);
		addRunnableTrigger(() -> this.driverReefScoringLevel = 4, () -> DRIVER_CONTROLLER.getPOV() == UP_DPAD);

		// // Toggling which level to score at (auto align)
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 1, () -> OPERATOR_CONTROLLER.getRawButtonPressed(1));
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 2, () -> OPERATOR_CONTROLLER.getRawButtonPressed(3));
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 3, () -> OPERATOR_CONTROLLER.getRawButtonPressed(4));
		addRunnableTrigger(() -> this.operatorReefScoringLevel = 4, () -> OPERATOR_CONTROLLER.getRawButtonPressed(5));

		// Togling which side of the hexagon to score at (auto align)
		addRunnableTrigger(() -> this.hexagonTargetSide = 1, () -> OPERATOR_CONTROLLER.getRawButtonPressed(11));
		addRunnableTrigger(() -> this.hexagonTargetSide = 2, () -> OPERATOR_CONTROLLER.getRawButtonPressed(12));
		addRunnableTrigger(() -> this.hexagonTargetSide = 3, () -> OPERATOR_CONTROLLER.getRawButtonPressed(7));
		addRunnableTrigger(() -> this.hexagonTargetSide = 4, () -> OPERATOR_CONTROLLER.getRawButtonPressed(8));
		addRunnableTrigger(() -> this.hexagonTargetSide = 5, () -> OPERATOR_CONTROLLER.getRawButtonPressed(9));
		addRunnableTrigger(() -> this.hexagonTargetSide = 6, () -> OPERATOR_CONTROLLER.getRawButtonPressed(10));

		// Toggling Left or Right Hexagon Side Scoring (auto align)
		addRunnableTrigger(() -> this.scoringReefLeft = true, () -> OPERATOR_CONTROLLER.getRawAxis(0) > AXIS_RECOGNITION_POINT);
		addRunnableTrigger(() -> this.scoringReefLeft = false, () -> OPERATOR_CONTROLLER.getRawAxis(0) < -AXIS_RECOGNITION_POINT);

		// // Climbing
		// addTrigger(ManagerStates.IDLE, ManagerStates.CLIMBING, () -> DRIVER_CONTROLLER.getLeftTriggerAxis() > 0.5);
		// addTrigger(ManagerStates.CLIMBING, ManagerStates.IDLE, () -> DRIVER_CONTROLLER.getLeftTriggerAxis() == 0);

		// Intaking at Coral Station
		// AA
		// addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING_CORALER, () -> DRIVER_CONTROLLER.getLeftBumperButtonPressed() || DRIVER_CONTROLLER.getRightBumperButtonPressed());
		// addTrigger(ManagerStates.INTAKING_CORALER, ManagerStates.INTAKING_CORALER_AA_OFF, autoAlign::nearTarget);
		// Manual
		addTrigger(SubsystemManagerStates.IDLE, SubsystemManagerStates.INTAKING_CORALER_AA_OFF, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(SubsystemManagerStates.INTAKING_CORALER_AA_OFF, SubsystemManagerStates.CENTERING_CORALER, () -> DRIVER_CONTROLLER.getXButtonPressed() || coraler.currentSenseGamepiece());
		addTrigger(SubsystemManagerStates.CENTERING_CORALER, SubsystemManagerStates.IDLE, () -> DRIVER_CONTROLLER.getXButtonPressed() || getStateTime() > CORAL_CENTERING_TIME);

		// // Intaking Algae
		// addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING_ALGAE_LOW, DRIVER_CONTROLLER::getBButtonPressed);
		// addTrigger(ManagerStates.INTAKING_ALGAE_HIGH, ManagerStates.INTAKING_ALGAE_LOW, () -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD);
		// addTrigger(ManagerStates.INTAKING_ALGAE_LOW, ManagerStates.INTAKING_ALGAE_HIGH, () -> DRIVER_CONTROLLER.getPOV() == UP_DPAD);
		// addTrigger(ManagerStates.INTAKING_ALGAE_HIGH, ManagerStates.HOLDING_ALGAE, DRIVER_CONTROLLER::getBButtonPressed);
		// addTrigger(ManagerStates.INTAKING_ALGAE_LOW, ManagerStates.HOLDING_ALGAE, DRIVER_CONTROLLER::getBButtonPressed);

		// // Scoring Algae at Processor
		// addTrigger(ManagerStates.HOLDING_ALGAE, ManagerStates.GOING_PROCESSOR, DRIVER_CONTROLLER::getAButtonPressed);
		// addTrigger(ManagerStates.GOING_PROCESSOR, ManagerStates.SCORING_PROCESSOR, () -> elevator.nearTarget() && algaer.nearTarget() && DRIVER_CONTROLLER.getAButtonPressed());
		// addTrigger(ManagerStates.SCORING_PROCESSOR, ManagerStates.IDLE, DRIVER_CONTROLLER::getAButtonPressed);

		// Scoring Reef Manual
		addTrigger(SubsystemManagerStates.IDLE, SubsystemManagerStates.TRANSITIONING_SCORING_REEF, () -> DRIVER_CONTROLLER.getPOV() != -1);
		addTrigger(SubsystemManagerStates.TRANSITIONING_SCORING_REEF, SubsystemManagerStates.SCORING_REEF_MANUAL, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(SubsystemManagerStates.SCORING_REEF_MANUAL, SubsystemManagerStates.IDLE, DRIVER_CONTROLLER::getYButtonPressed);
		// Auto
		addTrigger(SubsystemManagerStates.SCORING_REEF_MANUAL, SubsystemManagerStates.IDLE, () -> DriverStation.isAutonomous() && getStateTime() > SCORING_TIME);
		// Scoring Reef Auto Align
		addTrigger(SubsystemManagerStates.IDLE, SubsystemManagerStates.AUTO_ALIGN_FAR, () -> OPERATOR_CONTROLLER.getRawButtonPressed(2));
		addTrigger(SubsystemManagerStates.AUTO_ALIGN_FAR, SubsystemManagerStates.AUTO_ALIGN_CLOSE, autoAlign::readyForClose);
		addTrigger(SubsystemManagerStates.AUTO_ALIGN_CLOSE, SubsystemManagerStates.SCORING_REEF_AA, autoAlign::nearTarget);
		addTrigger(SubsystemManagerStates.SCORING_REEF_AA, SubsystemManagerStates.IDLE, DRIVER_CONTROLLER::getYButtonPressed);

		// // Zero Elevator
		// addTrigger(ManagerStates.IDLE, ManagerStates.ZEROING_ELEVATOR, () -> DRIVER_CONTROLLER.getBackButtonPressed());
		// addTrigger(ManagerStates.ZEROING_ELEVATOR, ManagerStates.IDLE, () -> DRIVER_CONTROLLER.getBackButtonPressed() || elevator.motorsZeroed());
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

	public void setLeftSourceTargeted(boolean leftSourceTargeted) {
		this.leftSourceSelected = leftSourceTargeted;
	}

	@Override
	public void runState() {
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/State Time", getStateTime());
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/State String", getState().getStateString());
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Selected Reef Side", hexagonTargetSide);
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Selected Reef Level", operatorReefScoringLevel);
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Left Pose Selected", scoringReefLeft);
		Logger.recordOutput(SubsystemManagerConstants.SUBSYSTEM_NAME + "/Driver Reef Level", driverReefScoringLevel);
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "RED");
		} else {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "BLUE");
		}

		// Set States, drive and vision are rogue so you don't need to set state
		elevator.setState(getState().getElevatorStateSupplier().get());
		coraler.setState(getState().getCoralerState());
		// algaer.setState(getState().getAlgaerState());
		autoAlign.setState(getState().getAutoAlignSupplier().get());
		ledSubsystem.setState(getState().getLedState());
		// climber.setState(getState().getClimberState());

		// Periodics
		autoAlign.periodic();
		elevator.periodic();
		coraler.periodic();
		// algaer.periodic();
		vision.periodic();
		drive.periodic();
		ledSubsystem.periodic();
		// climber.periodic();

		// STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!
		if (DRIVER_CONTROLLER.getBackButtonPressed() || OPERATOR_CONTROLLER.getRawButtonPressed(6)) {
			setState(SubsystemManagerStates.IDLE);
		}
	}
}
