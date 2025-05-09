package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.SIMULATION_PERIOD;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Sim.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

	private ProfiledPIDController pidController;
	private ElevatorFeedforward ffcontroller;
	private final ElevatorSim elevatorSim = new ElevatorSim(GEARBOX, GEARING, CARRIAGE_MASS.in(Kilograms), DRUM_RADIUS.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters), SIMULATE_GRAVITY, 0);

	private TalonFX leftMotor;
	private TalonFX rightMotor;
	private TalonFXSimState leftMotorSim;
	private TalonFXSimState rightMotorSim;

	private double appliedVoltage;
	private boolean zeroed;

	public ElevatorIOSim() {
		pidController = new ProfiledPIDController(PROFILLED_PID_CONSTANTS.p(), PROFILLED_PID_CONSTANTS.i(), PROFILLED_PID_CONSTANTS.d(), ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
		pidController.setTolerance(ElevatorConstants.POSITION_TOLERANCE.in(Meters), ElevatorConstants.VELOCITY_TOLERANCE.in(MetersPerSecond));
		pidController.setIZone(PROFILLED_PID_CONSTANTS.iZone());

		ffcontroller = new ElevatorFeedforward(FF_CONSTANTS.kS, FF_CONSTANTS.kG, FF_CONSTANTS.kV, FF_CONSTANTS.kA);
		zeroed = false;

		leftMotor = new TalonFX(LEFT_MOTOR_CANID);
		rightMotor = new TalonFX(RIGHT_MOTOR_CANID);
		leftMotorSim = leftMotor.getSimState();
		rightMotorSim = rightMotor.getSimState();
	}

	@Override
	public Distance getHeight() {
		return Meters.of(elevatorSim.getPositionMeters());
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		inputs.currentElevatorHeight = leftMotor.getPosition().getValue().in(Rotations) * METERS_PER_ROTATION.in(Meters);
		inputs.elevatorVelocity = leftMotor.getVelocity().getValue().in(RotationsPerSecond) * METERS_PER_ROTATION.in(Meters);

		inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().velocity;
		inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().position;

		inputs.leftMotorVoltInput = appliedVoltage;
		inputs.rightMotorVoltInput = appliedVoltage;
		inputs.elevatorZeroed = zeroed;

		leftMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters() / METERS_PER_ROTATION.in(Meters));
		leftMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() / METERS_PER_ROTATION.in(Meters));
		rightMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters() / METERS_PER_ROTATION.in(Meters));
		rightMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() / METERS_PER_ROTATION.in(Meters));
		leftMotorSim.setSupplyVoltage(appliedVoltage);
		rightMotorSim.setSupplyVoltage(appliedVoltage);

		pidController.setTolerance(POSITION_TOLERANCE_SIM.in(Meters), VELOCITY_TOLERANCE_SIM.in(MetersPerSecond));
	}

	@Override
	public void setHeightGoalpoint(Distance height) {
		pidController.setGoal(new State(height.in(Meters), 0));
	}

	@Override
	public void runElevator() {
		appliedVoltage = pidController.calculate(elevatorSim.getPositionMeters()) + ffcontroller.calculate(pidController.getSetpoint().velocity);
		elevatorSim.setInput(appliedVoltage);
		elevatorSim.update(SIMULATION_PERIOD);
		Logger.recordOutput("Elevator/applied volts", appliedVoltage);
		Logger.recordOutput("Elevator/Current Draw", elevatorSim.getCurrentDrawAmps());
	}

	@Override
	public boolean nearTarget() {
		return pidController.atGoal();
	}

	@Override
	public void zero() {
		elevatorSim.setState(0, 0);
		elevatorSim.update(SIMULATION_PERIOD);
	}

	@Override
	public void resetMotorsZeroed() {
		return;
	}

	@Override
	public boolean motorsZeroed() {
		return true;
	}

	@Override
	public Distance getStageOneHeight() {
		return Meters.of(elevatorSim.getPositionMeters());
	}

	@Override
	public Distance getCarriageHeight() {
		return Meters.of(elevatorSim.getPositionMeters() * 2);
	}

	@Override
	public TalonFX getLeftMotor() {
		return leftMotor;
	}

	@Override
	public TalonFX getRightMotor() {
		return rightMotor;
	}

	@Override
	public void resetController() {
		pidController.reset(leftMotor.getPosition().getValueAsDouble() * METERS_PER_ROTATION.in(Meters));
	}

	@Override
	public void zeroing() {
		elevatorSim.setInput(ZEROING_SPEED);
		elevatorSim.update(SIMULATION_PERIOD);
	}

	@Override
	public boolean nearZero() {
		return Math.abs(getHeight().in(Meters)) < 0.5;
	}
}
