package frc.robot.Subsystems.Manager;

import frc.robot.Subsystems.Algaer.Algaer;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Subsystems.Manager.ManagerConstants.*;

public class Manager extends Subsystem<ManagerStates> {

	private static Manager instance;

	private final Drive drive = Drive.getInstance();
	private final Elevator elevator = Elevator.getInstance();
	private final Coraler coraler = Coraler.getInstance();
	private final Algaer algaer = Algaer.getInstance();
	private final AutoAlign autoAlign = AutoAlign.getInstance();

	private Manager() {
		super("Manager", ManagerStates.IDLE);
		
		// Climbing
		// TODO: Check with yussuf if holding down the trigger for letting up the climber then releasing is good
		addTrigger(ManagerStates.IDLE, ManagerStates.CLIMBING, () -> DRIVER_CONTROLLER.getLeftTriggerAxis() > 0.5);
		addTrigger(ManagerStates.CLIMBING, ManagerStates.IDLE, () -> DRIVER_CONTROLLER.getLeftTriggerAxis() == 0);

		// Intaking at Coral Station
		// TODO: Cache left or right bumper presses in a variable for AA to the correct source
		addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING_CORALER, () -> DRIVER_CONTROLLER.getLeftBumperButtonPressed() || DRIVER_CONTROLLER.getRightBumperButtonPressed());
		addTrigger(ManagerStates.INTAKING_CORALER, ManagerStates.IDLE, () -> DRIVER_CONTROLLER.getLeftBumperButtonReleased() && DRIVER_CONTROLLER.getRightBumperButtonReleased());

		// Intaking Algae
		addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING_ALGAE_LOW, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ManagerStates.INTAKING_ALGAE_HIGH, ManagerStates.INTAKING_ALGAE_LOW, () -> DRIVER_CONTROLLER.getPOV() == DOWN_DPAD);
		addTrigger(ManagerStates.INTAKING_ALGAE_LOW, ManagerStates.INTAKING_ALGAE_HIGH, () -> DRIVER_CONTROLLER.getPOV() == UP_DPAD);
		addTrigger(ManagerStates.INTAKING_ALGAE_HIGH, ManagerStates.IDLE, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ManagerStates.INTAKING_ALGAE_LOW, ManagerStates.IDLE, DRIVER_CONTROLLER::getBButtonPressed);

		// Scoring Algae at Processor
		addTrigger(ManagerStates.IDLE, ManagerStates.GOING_PROCESSOR, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(ManagerStates.GOING_PROCESSOR, ManagerStates.SCORING_PROCESSOR, () -> elevator.nearTarget() && algaer.nearTarget());
		addTrigger(ManagerStates.SCORING_PROCESSOR, ManagerStates.IDLE, DRIVER_CONTROLLER::getAButtonPressed);

		// Scoring Reef Manual
		addTrigger(ManagerStates.IDLE, ManagerStates.TRANSITIONING_SCORING_REEF, () -> DRIVER_CONTROLLER.getPOV() != -1);
		addTrigger(ManagerStates.TRANSITIONING_SCORING_REEF, ManagerStates.SCORING_REEF, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.SCORING_REEF, ManagerStates.IDLE, DRIVER_CONTROLLER::getYButtonPressed);

		// Scoring Reef Auto Align
		// TODO: Implement operator input & make near target scale to setpoint
		addTrigger(ManagerStates.IDLE, ManagerStates.AUTO_ALIGN_FAR, () -> false);
		addTrigger(ManagerStates.AUTO_ALIGN_FAR, ManagerStates.AUTO_ALIGN_CLOSE, autoAlign::readyForClose);
		addTrigger(ManagerStates.AUTO_ALIGN_CLOSE, ManagerStates.SCORING_REEF, autoAlign::nearTarget);
		addTrigger(ManagerStates.SCORING_REEF, ManagerStates.IDLE, DRIVER_CONTROLLER::getYButtonPressed);
	}

	public static Manager getInstance() {
		if (instance == null) {
			instance = new Manager();
		}
		return instance;
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

		if (DRIVER_CONTROLLER.getXButtonPressed()) {
			setState(ManagerStates.IDLE);
		}
	}
}
