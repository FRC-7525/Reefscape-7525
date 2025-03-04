package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
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
			io.zero();
			return;
		}

		io.updateInputs(inputs);
		Logger.processInputs(ElevatorConstants.SUBSYSTEM_NAME, inputs);

		io.setHeightGoalpoint(getState().getTargetHeight());
		io.runElevator();

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
}
