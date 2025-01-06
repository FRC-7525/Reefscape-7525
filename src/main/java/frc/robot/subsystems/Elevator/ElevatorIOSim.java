package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.subsystems.Elevator.ElevatorConstants.Sim.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
		elevatorSim = new ElevatorSim(
			GEARBOX,
			GEARING,
			CARRIAGE_MASS.in(Kilograms),
			DRUM_RADIUS.in(Meters),
			MIN_HEIGHT.in(Meters),
			MAX_HEIGHT.in(Meters),
			SIMULATE_GRAVITY,
			STARTING_HEIGHT.in(Meters)
		);

		pidController = new ProfiledPIDController(
			PROFILLED_PID_CONSTANTS.kP,
			PROFILLED_PID_CONSTANTS.kI,
			PROFILLED_PID_CONSTANTS.kD,
			ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS
		);
		pidController.setTolerance(
			ElevatorConstants.POSITION_TOLERANCE.in(Meters),
			ElevatorConstants.VELOCITY_TOLERANCE.in(MetersPerSecond)
		);
		pidController.setIZone(PROFILLED_PID_CONSTANTS.iZone);

		ffcontroller = new ElevatorFeedforward(
			FF_CONSTANTS.kS,
			FF_CONSTANTS.kG,
			FF_CONSTANTS.kV,
			FF_CONSTANTS.kA
		);
		zeroed = false;

		leftMotor = new TalonFX(LEFT_MOTOR_CANID);
		rightMotor = new TalonFX(RIGHT_MOTOR_CANID);
		leftMotorSim = leftMotor.getSimState();
		rightMotorSim = rightMotor.getSimState();

		metersPerRotation = METERS_PER_ROTATION.in(Meters);
	}

	public void updateInputs(ElevatorIOInputs inputs) {
		elevatorSim.update(GlobalConstants.SIM_DELTA_TIME);

		inputs.currentElevatorHeight = elevatorSim.getPositionMeters();
		inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().position;

		inputs.elevatorVelocity = elevatorSim.getVelocityMetersPerSecond();
		inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
		inputs.elevatorVelocityGoalpoint = pidController.getGoal().velocity;

		inputs.leftMotorVoltInput = appliedVoltage;
		inputs.rightMotorVoltInput = appliedVoltage;
		inputs.elevatorZeroed = zeroed;

		leftMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters() / metersPerRotation);
		leftMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() / metersPerRotation);
		rightMotorSim.setRawRotorPosition(-elevatorSim.getPositionMeters() / metersPerRotation); // negative bc right is inversed (probably)
		rightMotorSim.setRotorVelocity(
			-elevatorSim.getVelocityMetersPerSecond() / metersPerRotation
		);
	}

	@Override
	public void setHeightGoalpoint(double height) {
		pidController.setGoal(height);
	}

	public void runElevator() {
		appliedVoltage =
			pidController.calculate(elevatorSim.getPositionMeters()) +
			ffcontroller.calculate(pidController.getSetpoint().velocity);
		elevatorSim.setInputVoltage(appliedVoltage);
	}

	@Override
	public boolean nearTarget() {
		return pidController.atGoal();
	}

	@Override
	public void zero() {
		zeroed = true;
	}

	@Override
	public boolean isZeroed() {
		return zeroed;
	}
}
