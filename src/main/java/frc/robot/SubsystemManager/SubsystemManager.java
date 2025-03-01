package frc.robot.SubsystemManager;

import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.SubsystemManager.SubsystemManagerConstants.*;
import static frc.robot.SubsystemManager.SubsystemManagerStates.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems.Algaer.Algaer;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.Vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.team7525.misc.Tracer;
import org.team7525.subsystem.Subsystem;

public class SubsystemManager extends Subsystem<SubsystemManagerStates> {

	private static SubsystemManager instance = new SubsystemManager();

	private final Drive drive = Drive.getInstance();
	// private final Climber climber = Climber.getInstance();
	private final Elevator elevator = Elevator.getInstance();
	private final Coraler coraler = Coraler.getInstance();
	private final AutoAlign autoAlign = AutoAlign.getInstance();
	private final Vision vision = Vision.getInstance();
	private final Algaer algaer = Algaer.getInstance();
	private final LED ledSubsystem = LED.getInstance();

	public Boolean leftSourceSelected = false;

	public int driverReefScoringLevel = 3;
	public int operatorReefScoringLevel = 1;
	public int hexagonTargetSide = 1;
	public boolean scoringReefLeft = false;

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

		// // Climbing
		// addTrigger(SubsystemManagerStates.IDLE, SubsystemManagerStates.CLIMBING, () -> DRIVER_CONTROLLER.getLeftTriggerAxis() > 0.5);
		// addTrigger(SubsystemManagerStates.CLIMBING, SubsystemManagerStates.IDLE, () -> DRIVER_CONTROLLER.getLeftTriggerAxis() == 0);

		// Intaking at Coral Station
		// AA
		addTrigger(IDLE, INTAKING_CORALER, () -> {
			boolean left = DRIVER_CONTROLLER.getLeftBumperButtonPressed();
			boolean right = DRIVER_CONTROLLER.getRightBumperButtonPressed();
			this.leftSourceSelected = left ? true : (right ? false : leftSourceSelected);
			return left || right;
		});
		addTrigger(INTAKING_CORALER, INTAKING_CORALER_AA_OFF, autoAlign::nearGoal);

		// Manual
		addTrigger(IDLE, INTAKING_CORALER_AA_OFF, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(INTAKING_CORALER_AA_OFF, IDLE, () -> DRIVER_CONTROLLER.getXButtonPressed() || coraler.hasGamepiece());

		// // Intaking Algae
		addTrigger(IDLE, INTAKING_ALGAE_GROUND, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(INTAKING_ALGAE_GROUND, INTAKING_ALGAE_LOW, () -> DRIVER_CONTROLLER.getPOV() == LEFT_DPAD || DRIVER_CONTROLLER.getPOV() == RIGHT_DPAD);
		addTrigger(INTAKING_ALGAE_GROUND, INTAKING_ALGAE_HIGH, () -> DRIVER_CONTROLLER.getPOV() == UP_DPAD);
		addTrigger(INTAKING_ALGAE_LOW, INTAKING_ALGAE_HIGH, () -> DRIVER_CONTROLLER.getPOV() == UP_DPAD);
		addTrigger(INTAKING_ALGAE_LOW, INTAKING_ALGAE_GROUND, () -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD);
		addTrigger(INTAKING_ALGAE_HIGH, INTAKING_ALGAE_LOW, () -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD);
		addTrigger(INTAKING_ALGAE_HIGH, INTAKING_ALGAE_LOW, () -> DRIVER_CONTROLLER.getPOV() == LEFT_DPAD || DRIVER_CONTROLLER.getPOV() == RIGHT_DPAD);

		// Go Back Down after intaking
		addTrigger(INTAKING_ALGAE_HIGH, HOLDING_ALGAE, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(INTAKING_ALGAE_LOW, HOLDING_ALGAE, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(INTAKING_ALGAE_GROUND, HOLDING_ALGAE, DRIVER_CONTROLLER::getBButtonPressed);

		// Scoring Algae at Processor
		addTrigger(HOLDING_ALGAE, GOING_PROCESSOR, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(GOING_PROCESSOR, SCORING_PROCESSOR, () -> (algaer.nearTarget() && DRIVER_CONTROLLER.getAButtonPressed()));
		addTrigger(SCORING_PROCESSOR, IDLE, DRIVER_CONTROLLER::getAButtonPressed);

		// Scoring Reef Manual
		addTrigger(IDLE, TRANSITIONING_SCORING_REEF, () -> DRIVER_CONTROLLER.getPOV() != -1);
		addTrigger(TRANSITIONING_SCORING_REEF, SCORING_REEF_MANUAL, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(SCORING_REEF_MANUAL, IDLE, DRIVER_CONTROLLER::getYButtonPressed);
		// Auto ONLY transition
		addTrigger(SCORING_REEF_MANUAL, IDLE, () -> DriverStation.isAutonomous() && getStateTime() > SCORING_TIME);

		// Scoring Reef Auto Align
		// See if odo is good enough to ALWAYS automatically score, otherwise we just have driver click y after minior adjustments
		addTrigger(IDLE, AUTO_ALIGN_FAR, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(AUTO_ALIGN_FAR, AUTO_ALIGN_CLOSE, autoAlign::readyForClose);
		addTrigger(AUTO_ALIGN_CLOSE, TRANSITIONING_SCORING_REEF, autoAlign::nearGoal);
		// Zero Elevator
		// TODO: Test
		// addTrigger(IDLE, ZEROING_ELEVATOR, () -> {
		// 	boolean pressed = OPERATOR_CONTROLLER.getRawButtonPressed(4);
		// 	if (pressed) {
		// 		elevator.resetMotorsZeroed();
		// 	}
		// 	return pressed;
		// });
		// addTrigger(ZEROING_ELEVATOR, IDLE, () -> () -> OPERATOR_CONTROLLER.getRawButtonPressed(4) || elevator.motorsZeroed());
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
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "RED");
		} else {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "BLUE");
		}

		// Set States, drive and vision are rogue so you don't need to set state
		elevator.setState(getState().getElevatorStateSupplier().get());
		coraler.setState(getState().getCoralerState());
		algaer.setState(getState().getAlgaerState());
		autoAlign.setState(getState().getAutoAlignSupplier().get());
		ledSubsystem.setState(getState().getLedStateSupplier().get());
		// climber.setState(getState().getClimberState());

		// Periodics
		Tracer.traceFunc("AutoAlignPeriodic", autoAlign::periodic);
		Tracer.traceFunc("ElevatorPeriodic", elevator::periodic);
		Tracer.traceFunc("CoralerPeriodic", coraler::periodic);
		Tracer.traceFunc("VisionPeriodic", vision::periodic);
		Tracer.traceFunc("AlgaerPeriodic", algaer::periodic);
		Tracer.traceFunc("DrivePeriodic", drive::periodic);
		Tracer.traceFunc("LEDPeriodic", ledSubsystem::periodic);
		// Tracer.traceFunc("ClimberPeriodic", climber::periodic);

		// STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!
		if (DRIVER_CONTROLLER.getBackButtonPressed() || OPERATOR_CONTROLLER.getRawButtonPressed(5)) { // 5
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
}
