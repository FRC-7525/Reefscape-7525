package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Coraler.CoralerConstants.Sim;

public class CoralerIOSim implements CoralerIO {

	private DCMotorSim motorSim;
	private SparkMaxSim sparkSim;
	private SparkMax spark;
	private PIDController wheelController;
	private double speedPoint;

	public CoralerIOSim() {
		motorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(Sim.NUM_MOTORS),
				Sim.MOTOR_MOI,
				GEARING
			),
			DCMotor.getNEO(Sim.NUM_MOTORS)
		);
		wheelController = WHEEL_CONTROLLER.get();
		spark = new SparkMax(Real.WHEEL_MOTOR_CAN_ID, MotorType.kBrushless);
		sparkSim = new SparkMaxSim(spark, DCMotor.getNEO(Sim.NUM_MOTORS));
		speedPoint = 0.0;
		sparkSim.enable();
	}

	@Override
	public void updateInputs(CoralerIOInputs inputs) {
		inputs.velocityRPS = sparkSim.getVelocity();
		inputs.speedPointRPS = speedPoint;

		motorSim.update(GlobalConstants.SIMULATION_PERIOD);
		sparkSim.setVelocity(motorSim.getAngularVelocityRPM() / 60);
		sparkSim.setPosition(motorSim.getAngularPositionRotations());
	}

	@Override
	public void setVelocity(AngularVelocity speedPoint) {
		this.speedPoint = speedPoint.in(RotationsPerSecond);
		motorSim.setInputVoltage(
			wheelController.calculate(sparkSim.getVelocity(), speedPoint.in(RotationsPerSecond))
		);
	}
}
