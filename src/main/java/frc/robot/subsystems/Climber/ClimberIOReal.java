package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.Real.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class ClimberIOReal implements ClimberIO {

	private SparkMax motor;
	private RelativeEncoder motorEncoder;

	private PIDController pidController;

	private LinearFilter filter;

	private double metersPerRotation;
	private boolean motorZeroed;
	private double setpoint;

	public ClimberIOReal() {
		motor = new SparkMax(MOTOR_CANID, MotorType.kBrushless);
		motorEncoder = motor.getEncoder();

		setpoint = 0;
		motorZeroed = false;
		metersPerRotation = METERS_PER_ROTATION.in(Meters);

		filter = LinearFilter.movingAverage(CURRENT_FILTER_TAPS);

		pidController = new PIDController(PID_CONSTANTS.kP, PID_CONSTANTS.kI, PID_CONSTANTS.kD);

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Climber PID controller", pidController);
		}
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberPosition = motorEncoder.getPosition() * metersPerRotation;
		inputs.climberSpeed = motorEncoder.getVelocity() / 60;
		inputs.climberAngularPosition = Units.rotationsToDegrees(motorEncoder.getPosition());
		inputs.climberHeightPoint = setpoint;
	}

	@Override
	public void setSetpoint(Distance setpoint) {
		double height = setpoint.in(Meters);
		this.setpoint = height;

		double voltage = pidController.calculate((motorEncoder.getPosition() * metersPerRotation), height);
		motor.setVoltage(voltage);
	}

	@Override
	public boolean nearSetpoint() {
		return Math.abs((motorEncoder.getPosition() * metersPerRotation) - setpoint) < POSITION_TOLERANCE.in(Meters);
	}

	@Override
	public void zero() {
		double zeroingSpeed = -ZEROING_VELOCITY.in(MetersPerSecond);

		if (filter.calculate(motor.getOutputCurrent()) > ZEROING_CURRENT_LIMIT.in(Amps) || motorZeroed) {
			if (!motorZeroed) motorEncoder.setPosition(0);
			setpoint = IDLE.in(Meters);
			zeroingSpeed = pidController.calculate(motorEncoder.getPosition() * metersPerRotation, setpoint);
			motorZeroed = true;
		}

		motor.setVoltage(zeroingSpeed);
	}

	@Override
	public boolean isZeroed() {
		return motorZeroed;
	}

	@Override
	public void stop() {
		motor.setVoltage(0);
	}
}
