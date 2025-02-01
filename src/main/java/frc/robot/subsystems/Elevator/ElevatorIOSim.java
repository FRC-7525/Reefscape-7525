package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Sim.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.GlobalConstants;

public class ElevatorIOSim implements ElevatorIO {

	private ProfiledPIDController pidController;
	private ElevatorFeedforward ffcontroller;
	private ElevatorSim elevatorSim;

	private TalonFX leftMotor;
	private TalonFX rightMotor;
	private TalonFXSimState leftMotorSim;
	private TalonFXSimState rightMotorSim;

	private double appliedVoltage;
	private boolean zeroed;
	public double metersPerRotation;

	public ElevatorIOSim() {
		elevatorSim = new ElevatorSim(GEARBOX, GEARING, CARRIAGE_MASS.in(Kilograms), DRUM_RADIUS.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters), SIMULATE_GRAVITY, STARTING_HEIGHT.in(Meters));

		pidController = new ProfiledPIDController(PROFILLED_PID_CONSTANTS.kP, PROFILLED_PID_CONSTANTS.kI, PROFILLED_PID_CONSTANTS.kD, ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
		pidController.setTolerance(ElevatorConstants.POSITION_TOLERANCE.in(Meters), ElevatorConstants.VELOCITY_TOLERANCE.in(MetersPerSecond));
		pidController.setIZone(PROFILLED_PID_CONSTANTS.iZone);

		ffcontroller = new ElevatorFeedforward(FF_CONSTANTS.kS, FF_CONSTANTS.kG, FF_CONSTANTS.kV, FF_CONSTANTS.kA);
		zeroed = false;

		leftMotor = new TalonFX(LEFT_MOTOR_CANID);
		rightMotor = new TalonFX(RIGHT_MOTOR_CANID);
		leftMotorSim = leftMotor.getSimState();
		rightMotorSim = rightMotor.getSimState();

		metersPerRotation = METERS_PER_ROTATION.in(Meters);
	}

	public void updateInputs(ElevatorIOInputs inputs) {
		elevatorSim.update(GlobalConstants.SIMULATION_PERIOD);

		inputs.currentElevatorHeight = elevatorSim.getPositionMeters() * METERS_PER_ROTATION.in(Meters);
		inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().position;

		inputs.elevatorVelocity = elevatorSim.getVelocityMetersPerSecond() * METERS_PER_ROTATION.in(Meters);
		inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
		inputs.elevatorVelocityGoalpoint = pidController.getGoal().velocity;

		inputs.leftMotorVoltInput = appliedVoltage;
		inputs.rightMotorVoltInput = appliedVoltage;
		inputs.elevatorZeroed = zeroed;

		leftMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters() / METERS_PER_ROTATION.in(Meters));
		leftMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() / METERS_PER_ROTATION.in(Meters));
		rightMotorSim.setRawRotorPosition(-elevatorSim.getPositionMeters() / METERS_PER_ROTATION.in(Meters)); // negative bc right is inversed (probably)
		rightMotorSim.setRotorVelocity(-elevatorSim.getVelocityMetersPerSecond() / METERS_PER_ROTATION.in(Meters));
	}

	@Override
	public void setHeightGoalpoint(Distance height) {
		pidController.setGoal(height.in(Meters));
	}

	public void runElevator() {
		appliedVoltage = pidController.calculate(elevatorSim.getPositionMeters() * METERS_PER_ROTATION.in(Meters)) + ffcontroller.calculate(pidController.getSetpoint().velocity);
		elevatorSim.setInputVoltage(appliedVoltage);
	}

	@Override
	public boolean nearTarget() {
		return pidController.atGoal();
	}

	@Override
	public void zero() {
		return;
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
	public Distance getCarraigeHeight() {
		return Meters.of(elevatorSim.getPositionMeters() * 2 + Units.inchesToMeters(1));
	}
}
