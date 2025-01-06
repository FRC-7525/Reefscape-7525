package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.subsystems.Elevator.ElevatorConstants.Real.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class ElevatorIOReal implements ElevatorIO {

	private TalonFX leftMotor;
	private TalonFXConfigurator leftConfigurator;
	private TalonFXConfiguration leftConfigurations;

	private TalonFX rightMotor;
	private TalonFXConfigurator rightConfigurator;
	private TalonFXConfiguration rightConfigurations;

	private ProfiledPIDController pidController;
	private ElevatorFeedforward ffcontroller;
	private DigitalInput limitSwitch;

	private double metersPerRotation;
	private double leftMotorVoltage;
	private double rightMotorVoltage;
	private boolean leftMotorZeroed;
	private boolean rightMotorZeroed;

	public ElevatorIOReal() {
		leftMotor = new TalonFX(LEFT_MOTOR_CANID);
		rightMotor = new TalonFX(RIGHT_MOTOR_CANID);
		limitSwitch = new DigitalInput(LIMIT_SWITCH_DIO);

		leftMotorVoltage = 0;
		rightMotorVoltage = 0;
		leftMotorZeroed = false;
		rightMotorZeroed = false;
		metersPerRotation = METERS_PER_ROTATION.in(Meters);

		//Motor configs
		leftConfigurator = leftMotor.getConfigurator();
		rightConfigurator = rightMotor.getConfigurator();

		leftConfigurations = new TalonFXConfiguration();
		rightConfigurations = new TalonFXConfiguration();

		leftConfigurations.MotorOutput.Inverted = LEFT_INVERTED
			? InvertedValue.Clockwise_Positive
			: InvertedValue.CounterClockwise_Positive;
		leftConfigurations.MotorOutput.NeutralMode = LEFT_NEUTRAL_MODE;
		leftConfigurations.CurrentLimits.StatorCurrentLimitEnable =
			LEFT_STRATOR_CURRENT_LIMIT_ENABLED;
		leftConfigurations.CurrentLimits.StatorCurrentLimit =
			LEFT_STRATOR_CURRENT_LIMIT.in(Amps);
		leftConfigurator.apply(leftConfigurations);

		rightConfigurations.MotorOutput.Inverted = RIGHT_INVERTED
			? InvertedValue.Clockwise_Positive
			: InvertedValue.CounterClockwise_Positive;
		rightConfigurations.MotorOutput.NeutralMode = RIGHT_NEUTRAL_MODE;
		rightConfigurations.CurrentLimits.StatorCurrentLimitEnable =
			RIGHT_STRATOR_CURRENT_LIMIT_ENABLED;
		rightConfigurations.CurrentLimits.StatorCurrentLimit =
			RIGHT_STRATOR_CURRENT_LIMIT.in(Amps);
		rightConfigurator.apply(rightConfigurations);

		//PID and FF controller setup
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

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Elevator PID controller", pidController);
		}
	}

	@Override
	public void setHeightGoalpoint(double height) {
		pidController.setGoal(height);
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		inputs.currentElevatorHeight =
			leftMotor.getPosition().getValueAsDouble() * metersPerRotation;
		inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().position;
		inputs.elevatorVelocity = leftMotor.getVelocity().getValueAsDouble();
		inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().velocity;
		inputs.leftMotorVoltInput = leftMotorVoltage;
		inputs.rightMotorVoltInput = rightMotorVoltage;
	}

	@Override
	public void runElevator() {
		leftMotorVoltage =
			pidController.calculate(
				leftMotor.getPosition().getValueAsDouble() * metersPerRotation
			) +
			ffcontroller
				.calculate(pidController.getSetpoint().velocity);
		rightMotorVoltage =
			pidController.calculate(
				rightMotor.getPosition().getValueAsDouble() * metersPerRotation
			) +
			ffcontroller.calculate(pidController.getSetpoint().velocity);
		leftMotor.setVoltage(leftMotorVoltage);
		rightMotor.setVoltage(rightMotorVoltage);
	}

	@Override
	public boolean nearTarget() {
		return pidController.atGoal();
	}

	@Override
	public void zero() {
		double leftZeroingSpeed = -ElevatorConstants.ZEROING_VELOCITY.in(MetersPerSecond);
		double rightZeroingSpeed = -ElevatorConstants.ZEROING_VELOCITY.in(MetersPerSecond);

		if (
			rightMotor.getStatorCurrent().getValueAsDouble() >
				ElevatorConstants.ZEROING_CURRENT_LIMIT.in(Amps) ||
			!limitSwitch.get()
		) {
			rightZeroingSpeed = 0;
			if (!rightMotorZeroed) rightMotor.setPosition(0);
			rightMotorZeroed = true;
		}

		if (
			leftMotor.getStatorCurrent().getValueAsDouble() >
				ElevatorConstants.ZEROING_CURRENT_LIMIT.in(Amps) ||
			!limitSwitch.get()
		) {
			leftZeroingSpeed = 0;
			if (!leftMotorZeroed) leftMotor.setPosition(0);
			leftMotorZeroed = true;
		}

		rightMotor.set(rightZeroingSpeed);
		leftMotor.set(leftZeroingSpeed);
	}

	@Override
	public boolean isZeroed() {
		return leftMotorZeroed && rightMotorZeroed;
	}
}
