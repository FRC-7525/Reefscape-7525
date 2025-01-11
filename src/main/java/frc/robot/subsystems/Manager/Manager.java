package frc.robot.Subsystems.Manager;

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
	private final AutoAlign autoAlign = AutoAlign.getInstance();

	private Manager() {
		super("Manager", ManagerStates.IDLE);
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
	}
}
