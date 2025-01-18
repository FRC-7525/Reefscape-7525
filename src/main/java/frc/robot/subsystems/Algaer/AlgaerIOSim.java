package frc.robot.Subsystems.Algaer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.SIMULATION_PERIOD;
import static frc.robot.Subsystems.Algaer.AlgaerConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AlgaerIOSim implements AlgaerIO {

	private SingleJointedArmSim pivotSim;

	private DCMotorSim wheelMotorSim;
	private SparkMax dummyWheelSpark;
	private SparkMax dummyPivotSpark;

	private SparkMaxSim wheelSparkSim;
	private SparkMaxSim pivotSparkSim;

	private PIDController pivotController;
	private PIDController wheelSpeedController;

	private double wheelSpeedSetpoint;
	private double pivotPositionSetpoint;

	public AlgaerIOSim() {
		pivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(AlgaerConstants.Sim.NUM_PIVOT_MOTORS),
			AlgaerConstants.Sim.PIVOT_GEARING,
			AlgaerConstants.Sim.PIVOT_MOI.in(KilogramSquareMeters),
			AlgaerConstants.Sim.PIVOT_ARM_LENGTH.in(Meters),
			AlgaerConstants.Sim.MIN_PIVOT_ANGLE.in(Radians),
			AlgaerConstants.Sim.MAX_PIVOT_ANGLE.in(Radians),
			false,
			AlgaerConstants.Sim.STARTING_PIVOT_ANGLE.in(Radians)
		);

		wheelMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(AlgaerConstants.Sim.NUM_WHEEL_MOTORS), AlgaerConstants.Sim.WHEEL_MOTOR_MOI.in(KilogramSquareMeters), AlgaerConstants.Sim.WHEEL_MOTOR_GEARING), DCMotor.getNEO(AlgaerConstants.Sim.NUM_WHEEL_MOTORS));

		dummyWheelSpark = new SparkMax(Real.WHEEL_MOTOR_CANID, MotorType.kBrushless);
		dummyPivotSpark = new SparkMax(Real.PIVOT_MOTOR_CANID, MotorType.kBrushless);
		wheelSparkSim = new SparkMaxSim(dummyWheelSpark, DCMotor.getNEO(AlgaerConstants.Sim.NUM_WHEEL_MOTORS));
		pivotSparkSim = new SparkMaxSim(dummyPivotSpark, DCMotor.getNEO(AlgaerConstants.Sim.NUM_PIVOT_MOTORS));

		pivotController = PIVOT_CONTROLLER.get();
		wheelSpeedController = WHEEL_CONTROLLER.get();

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

		wheelSparkSim.setVelocity(wheelMotorSim.getAngularVelocityRPM()/60);
		wheelSparkSim.setPosition(wheelMotorSim.getAngularPositionRotations());

		pivotSparkSim.setVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));
		pivotSparkSim.setPosition(Units.radiansToRotations(pivotSim.getAngleRads()));
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint.in(Degrees);
		pivotSim.setInputVoltage(pivotController.calculate(Units.radiansToDegrees(pivotSim.getAngleRads()), pivotSetpoint.in(Degrees)));
	}

	@Override
	public void setWheelSpeed(AngularVelocity wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint.in(RotationsPerSecond);
		wheelMotorSim.setInputVoltage(wheelSpeedController.calculate(Units.radiansToRotations(wheelMotorSim.getAngularVelocityRadPerSec()), wheelSpeedSetpoint.in(RotationsPerSecond)));
	}

	@Override
	public boolean nearTarget() {
		return ((Math.abs(Units.radiansToDegrees(pivotSim.getAngleRads()) - pivotPositionSetpoint) < PIVOT_TOLERANCE.in(Degrees)) && (Math.abs(Units.radiansToRotations(wheelMotorSim.getAngularVelocityRadPerSec()) - wheelSpeedSetpoint) < WHEEL_TOLERANCE.in(RotationsPerSecond)));
	}
}
