package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.Sim.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystems.Climber.ClimberConstants.Real;

public class ClimberIOSim implements ClimberIO {

	private DCMotorSim climberSim;
	private SparkMax dummySpark;
	private SparkSim climberSparkSim;

	private PIDController pidController;

	private double climberSetpoint;

	public ClimberIOSim() {
		climberSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(NUM_MOTORS), MOTOR_MOI.magnitude(), MOTOR_GEARING), DCMotor.getNEO(NUM_MOTORS));
		pidController = new PIDController(PID_CONSTANTS.kP, PID_CONSTANTS.kI, PID_CONSTANTS.kD);
		climberSetpoint = 0;

		dummySpark = new SparkMax(Real.CLIMBER_CANID, MotorType.kBrushless);
		climberSparkSim = new SparkSim(dummySpark, DCMotor.getNEO(NUM_MOTORS));
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberSpeed = climberSim.getAngularVelocityRPM() / 60;
		inputs.climberPosition = climberSim.getAngularPositionRotations() * METERS_PER_ROTATION.in(Meters);
		inputs.climberAngularPosition = Units.rotationsToDegrees(climberSim.getAngularPositionRotations());
		inputs.climberHeightPoint = climberSetpoint;

		climberSparkSim.setVelocity(climberSim.getAngularVelocityRPM() / 60);
		climberSparkSim.setPosition(climberSim.getAngularPositionRotations());
	}

	@Override
	public void setSetpoint(Distance setpoint) {
		double height = setpoint.in(Meters);

		climberSetpoint = height;
		double voltage = pidController.calculate(climberSim.getAngularPositionRotations() * METERS_PER_ROTATION.in(Meters), height);
		climberSim.setInputVoltage(voltage);
	}

	@Override
	public void stop() {
		climberSim.setInputVoltage(0);
	}

	@Override
	public Distance getPosition() {
		return Meters.of(climberSim.getAngularPositionRad() * METERS_PER_ROTATION.in(Meters));
	}

	@Override
	public boolean nearSetpoint() {
		return (Math.abs(climberSim.getAngularPositionRotations() - climberSetpoint) < POSITION_TOLERANCE.in(Meters));
	}
}
