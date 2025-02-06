package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.Real.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class ClimberIOReal implements ClimberIO {

	private SparkMax motor;
	private RelativeEncoder motorEncoder;

	private PIDController pidController;

	private Distance setpoint;

	public ClimberIOReal() {
		motor = new SparkMax(Real.CLIMBER_CANID, MotorType.kBrushless);
		motorEncoder = motor.getEncoder();
		setpoint = Meters.of(0);

		pidController = new PIDController(PID_CONSTANTS.kP, PID_CONSTANTS.kI, PID_CONSTANTS.kD);

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Climber PID controller", pidController);
		}
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberPosition = motorEncoder.getPosition() * METERS_PER_ROTATION.in(Meters);
		inputs.climberSpeed = (motorEncoder.getVelocity() / 60) * METERS_PER_ROTATION.in(Meters);
		inputs.climberAngularPosition = Units.rotationsToDegrees(motorEncoder.getPosition());
		inputs.climberHeightPoint = setpoint.in(Meters);
	}

	@Override
	public void setSetpoint(Distance setpoint) {
		this.setpoint = setpoint;

		double voltage = pidController.calculate((motorEncoder.getPosition() * METERS_PER_ROTATION.in(Meters)), setpoint.in(Meters));
		motor.setVoltage(voltage);
	}

	@Override
	public boolean nearSetpoint() {
		return (Math.abs((motorEncoder.getPosition() * METERS_PER_ROTATION.in(Meters)) - setpoint.in(Meters)) < POSITION_TOLERANCE.in(Meters));
	}

	@Override
	public Distance getPosition() {
		return Meters.of(motorEncoder.getPosition() * METERS_PER_ROTATION.in(Meters));
	}

	@Override
	public void stop() {
		motor.setVoltage(0);
	}
}
