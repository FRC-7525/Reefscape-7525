package frc.robot.Subsystems.Algaer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.SIMULATION_PERIOD;
import static frc.robot.Subsystems.Algaer.AlgaerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Subsystems.Algaer.AlgaerConstants.Real;

public class AlgaerIOSim implements AlgaerIO {

	private SingleJointedArmSim pivotSim;

	private DCMotorSim wheelMotorSim;
	private TalonFX wheelMotor;
	private TalonFX pivotMotor;

	private TalonFXSimState wheelMotorSimState;
	private TalonFXSimState pivotMotorSimState;

	private PIDController pivotController;

	private double wheelSpeedSetpoint;
	private double pivotPositionSetpoint;

	public AlgaerIOSim() {
		pivotSim = new SingleJointedArmSim(DCMotor.getNEO(AlgaerConstants.Sim.NUM_PIVOT_MOTORS), OVERALL_GEARING, Sim.PIVOT_MOI.in(KilogramSquareMeters), Sim.PIVOT_ARM_LENGTH.in(Meters), Sim.MIN_PIVOT_ANGLE.in(Radians), Sim.MAX_PIVOT_ANGLE.in(Radians), false, Sim.STARTING_PIVOT_ANGLE.in(Radians));

		wheelMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(AlgaerConstants.Sim.NUM_WHEEL_MOTORS), AlgaerConstants.Sim.WHEEL_MOTOR_MOI.in(KilogramSquareMeters), AlgaerConstants.Sim.WHEEL_MOTOR_GEARING), DCMotor.getKrakenX60(AlgaerConstants.Sim.NUM_WHEEL_MOTORS));

		wheelMotor = new TalonFX(Real.WHEEL_MOTOR_CANID);
		pivotMotor = new TalonFX(Real.PIVOT_MOTOR_CANID);
		wheelMotorSimState = new TalonFXSimState(wheelMotor);
		pivotMotorSimState = new TalonFXSimState(pivotMotor);

		pivotController = PIVOT_CONTROLLER.get();

		wheelSpeedSetpoint = 0;
		pivotPositionSetpoint = 0;
	}

	@Override
	public void updateInputs(AlgaerIOInputs input) {
		pivotSim.update(SIMULATION_PERIOD);
		wheelMotorSim.update(SIMULATION_PERIOD);

		input.wheelSpeed = Units.radiansToDegrees(wheelMotorSim.getAngularVelocityRadPerSec());
		input.wheelSpeedSetpoint = wheelSpeedSetpoint;
		input.pivotPosition = Units.radiansToDegrees(pivotSim.getAngleRads());
		input.pivotSetpoint = pivotPositionSetpoint;

		wheelMotorSimState.setRotorVelocity(wheelMotorSim.getAngularVelocity());
		wheelMotorSimState.setRawRotorPosition(wheelMotorSim.getAngularPosition());

		pivotMotorSimState.setRotorVelocity(RadiansPerSecond.of(pivotSim.getVelocityRadPerSec() / OVERALL_GEARING));
		pivotMotorSimState.setRawRotorPosition(Radians.of(pivotSim.getAngleRads() / OVERALL_GEARING));
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint.in(Degrees);
		pivotSim.setInputVoltage(pivotController.calculate(Units.radiansToDegrees(pivotSim.getAngleRads()), pivotSetpoint.in(Degrees)));
	}

	@Override
	public void setWheelSpeed(double wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint;
		wheelMotorSim.setInputVoltage(wheelSpeedSetpoint * 12);
	}

	@Override
	public boolean nearTarget() {
		return (Math.abs(Units.radiansToDegrees(pivotSim.getAngleRads()) - pivotPositionSetpoint) < PIVOT_TOLERANCE.in(Degrees));
	}

	@Override
	public Angle getAngle() {
		return Radians.of(pivotSim.getAngleRads());
	}

	@Override
	public TalonFX getWheelMotor() {
		return wheelMotor;
	}

	public SparkMax getPivotMotor() {
		return new SparkMax(1, MotorType.kBrushless);
	}
}
