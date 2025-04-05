package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Real.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

	private TalonFX leftMotor;
	private TalonFXConfigurator leftConfigurator;
	private TalonFXConfiguration leftConfigurations;

	private TalonFX rightMotor;
	private TalonFXConfigurator rightConfigurator;
	private TalonFXConfiguration rightConfigurations;

	private ProfiledPIDController pidController;
	private ElevatorFeedforward ffcontroller;

	private double leftMotorVoltage;
	private double rightMotorVoltage;

	private boolean leftMotorZeroed;
	private double timer = -1;

	public ElevatorIOReal() {
		leftMotor = new TalonFX(LEFT_MOTOR_CANID);
		rightMotor = new TalonFX(RIGHT_MOTOR_CANID);

		leftMotorVoltage = 0;
		rightMotorVoltage = 0;

		leftMotorZeroed = false;

		//Motor configs
		leftConfigurator = leftMotor.getConfigurator();
		rightConfigurator = rightMotor.getConfigurator();

		leftConfigurations = new TalonFXConfiguration();
		rightConfigurations = new TalonFXConfiguration();

		leftConfigurations.MotorOutput.Inverted = LEFT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		leftConfigurations.MotorOutput.NeutralMode = LEFT_NEUTRAL_MODE;
		leftConfigurations.CurrentLimits.StatorCurrentLimitEnable = LEFT_STRATOR_CURRENT_LIMIT_ENABLED;
		leftConfigurations.CurrentLimits.StatorCurrentLimit = LEFT_STRATOR_CURRENT_LIMIT.in(Amps);
		leftConfigurator.apply(leftConfigurations);

		rightConfigurations.MotorOutput.Inverted = RIGHT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		rightConfigurations.MotorOutput.NeutralMode = RIGHT_NEUTRAL_MODE;
		rightConfigurations.CurrentLimits.StatorCurrentLimitEnable = RIGHT_STRATOR_CURRENT_LIMIT_ENABLED;
		rightConfigurations.CurrentLimits.StatorCurrentLimit = RIGHT_STRATOR_CURRENT_LIMIT.in(Amps);
		rightConfigurator.apply(rightConfigurations);

		//PID and FF controller setup
		pidController = new ProfiledPIDController(PROFILLED_PID_CONSTANTS.p(), PROFILLED_PID_CONSTANTS.i(), PROFILLED_PID_CONSTANTS.d(), ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
		pidController.setTolerance(ElevatorConstants.POSITION_TOLERANCE.in(Meters), ElevatorConstants.VELOCITY_TOLERANCE.in(MetersPerSecond));
		pidController.setIZone(PROFILLED_PID_CONSTANTS.iZone());

		ffcontroller = new ElevatorFeedforward(FF_CONSTANTS.kS, FF_CONSTANTS.kG, FF_CONSTANTS.kV, FF_CONSTANTS.kA);

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Elevator PID controller", pidController);
		}
		SmartDashboard.putData("Elevator PID controller", pidController);

		leftMotor.setPosition(Degrees.of(0));
		rightMotor.setPosition(Degrees.of(0));
		pidController.setTolerance(POSITION_TOLERANCE.in(Meter));

		rightMotor.setControl(new Follower(LEFT_MOTOR_CANID, false));
	}

	@Override
	public void setHeightGoalpoint(Distance height) {
		pidController.setGoal(height.in(Meters));
	}

	@Override
	public Distance getHeight() {
		return METERS_PER_ROTATION.times(leftMotor.getPosition().getValueAsDouble());
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		inputs.currentElevatorHeight = leftMotor.getPosition().getValue().in(Rotations) * METERS_PER_ROTATION.in(Meters);
		inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().position;
		inputs.elevatorVelocity = leftMotor.getVelocity().getValue().in(RotationsPerSecond) * METERS_PER_ROTATION.in(Meters);
		inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().velocity;
		inputs.leftMotorVoltInput = leftMotorVoltage;
		inputs.rightMotorVoltInput = rightMotorVoltage;

		// For Tuning Only
		Logger.recordOutput(SUBSYSTEM_NAME + "/Left Encoder Pos", leftMotor.getPosition().getValue().in(Rotations));
		Logger.recordOutput(SUBSYSTEM_NAME + "/Right Encoder Pos", rightMotor.getPosition().getValue().in(Rotations));
		Logger.recordOutput(SUBSYSTEM_NAME + "/Left Motor Stator Current", leftMotor.getStatorCurrent().getValueAsDouble());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Right Motor Stator Current", rightMotor.getStatorCurrent().getValueAsDouble());
	}

	@Override
	public void runElevator() {
		leftMotorVoltage = pidController.calculate(leftMotor.getPosition().getValueAsDouble() * METERS_PER_ROTATION.in(Meters)) + ffcontroller.calculate(pidController.getSetpoint().velocity);
		leftMotor.setVoltage(leftMotorVoltage);
	}

	@Override
	public boolean nearTarget() {
		return pidController.atGoal();
	}

	@Override
	public void zero() {
		leftMotor.set(0);
		rightMotor.set(0);
		leftMotor.setPosition(0);
	}

	@Override
	public void resetMotorsZeroed() {
		return;
	}

	@Override
	public boolean motorsZeroed() {
		return leftMotorZeroed;
	}

	@Override
	public Distance getStageOneHeight() {
		return Meters.of(leftMotor.getPosition().getValueAsDouble() * METERS_PER_ROTATION.in(Meters));
	}

	@Override
	public Distance getCarriageHeight() {
		return Meters.of(2 * (leftMotor.getPosition().getValueAsDouble() * METERS_PER_ROTATION.in(Meters)));
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
		leftMotor.set(ZEROING_SPEED);
		rightMotor.set(ZEROING_SPEED);
	}

	@Override
	public boolean nearZero() {
		return Math.abs(getHeight().in(Meters)) < 0.5;
	}
}
