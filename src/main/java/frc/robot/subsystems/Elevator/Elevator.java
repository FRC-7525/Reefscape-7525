package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Elevator extends Subsystem<ElevatorStates> {

	private static Elevator instance;

	private ElevatorIO io;
	private ElevatorIOInputsAutoLogged inputs;

	private Elevator() {
		super(SUBSYSTEM_NAME, ElevatorStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new ElevatorIOSim();
			case REAL -> new ElevatorIOReal();
			case TESTING -> new ElevatorIOReal();
		};
		inputs = new ElevatorIOInputsAutoLogged();
	}

	public static Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}
		return instance;
	}

	@Override
	protected void runState() {
		if (getState() == ElevatorStates.ZEROING) {
			io.zeroing();
		} else {
			io.runElevator();
		}

		double leftAxis = DRIVER_CONTROLLER.getLeftTriggerAxis();
		double rightAxis = DRIVER_CONTROLLER.getRightTriggerAxis();

		//This way it guarantees that it will reset to 0 once you exit the state
		if (getState() != ElevatorStates.IDLE && TOGGLE_MANUAL_CONTROL) {
			//TODO: Could make this a runnable trigger in SubsystemManager but does it matter?
			if (leftAxis > TRIGGER_THRESHOLD) {
				io.setHeightGoalpoint(io.getHeight().plus(MANUAL_HEIGHT_CHANGE.times(2)));
			} else if (rightAxis > TRIGGER_THRESHOLD) {
				io.setHeightGoalpoint(io.getHeight().minus(MANUAL_HEIGHT_CHANGE));
			}
		}

		io.updateInputs(inputs);
		Logger.processInputs(ElevatorConstants.SUBSYSTEM_NAME, inputs);
		Logger.recordOutput("Elevator/Carraige Position", new Pose3d(new Translation3d(0, 0, io.getCarriageHeight().in(Meters)), new Rotation3d()));
		Logger.recordOutput("Elevator/Stage1 Position", new Pose3d(new Translation3d(0, 0, io.getStageOneHeight().in(Meters)), new Rotation3d()));
		Logger.recordOutput("Elevator/Carraige Height", io.getCarriageHeight().in(Meters));
		Logger.recordOutput("Elevator/Stage1 Height", io.getStageOneHeight());
		Logger.recordOutput("Elevator/State", getState().getStateString());
		Logger.recordOutput("Elevator/Near Setpoint", io.nearTarget());
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}

	public boolean nearEnoughTarget() {
		return io.getHeight().in(Inches) < NEAR_ENOUGH_POSITION.in(Inches);
	}

	public Distance getHeight() {
		return io.getHeight();
	}

	public boolean motorsZeroed() {
		return io.motorsZeroed();
	}

	public void resetMotorsZeroed() {
		io.resetMotorsZeroed();
	}

	public Distance getCarriageHeight() {
		return io.getCarriageHeight();
	}

	public TalonFX getLeftMotor() {
		return io.getLeftMotor();
	}

	public TalonFX getRightMotor() {
		return io.getRightMotor();
	}

	public double getStateTime() {
		return getStateTime();
	}

	@Override
	public void stateExit() {
		io.resetController();

		if (getState() == ElevatorStates.ZEROING) {
			io.zero();
		}
	}

	@Override
	protected void stateInit() {
		//Only sets height once so that manual control works better
		io.setHeightGoalpoint(getState().getTargetHeight());
	}
}
