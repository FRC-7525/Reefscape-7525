package frc.robot.subsystems.Algaer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.SIM_PERIOD;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
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
	private PIDController pivotController;
	private PIDController wheelSpeedController;

	private double wheelSpeedSetpoint;
	private double pivotPositionSetpoint;

	AlgaerIOSim() {
		pivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(AlgaerConstants.Sim.NUM_PIVOT_MOTORS),
			AlgaerConstants.Sim.PIVOT_GEARING,
			AlgaerConstants.Sim.PIVOT_MOI.magnitude(),
			AlgaerConstants.Sim.PIVOT_ARM_LENGTH.magnitude(),
			Units.degreesToRadians(AlgaerConstants.Sim.MIN_PIVOT_ANGLE.magnitude()),
			Units.degreesToRadians(AlgaerConstants.Sim.MAX_PIVOT_ANGLE.magnitude()),
			false,
			Units.degreesToRadians(AlgaerConstants.Sim.STARTING_PIVOT_ANGLE.magnitude())
		);

		wheelMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(AlgaerConstants.Sim.NUM_WHEEL_MOTORS),
				AlgaerConstants.Sim.WHEEL_MOTOR_MOI.magnitude(),
				AlgaerConstants.Sim.WHEEL_MOTOR_GEARING
			),
			DCMotor.getNEO(AlgaerConstants.Sim.NUM_WHEEL_MOTORS)
		);

		pivotController = new PIDController(
			AlgaerConstants.Sim.PIVOT_PID_CONSTANTS.kP,
			AlgaerConstants.Sim.PIVOT_PID_CONSTANTS.kI,
			AlgaerConstants.Sim.PIVOT_PID_CONSTANTS.kD
		);
		wheelSpeedController = new PIDController(
			AlgaerConstants.Sim.WHEEL_PID_CONSTANTS.kP,
			AlgaerConstants.Sim.WHEEL_PID_CONSTANTS.kI,
			AlgaerConstants.Sim.WHEEL_PID_CONSTANTS.kD
		);

		wheelSpeedSetpoint = 0;
		pivotPositionSetpoint = 0;
	}

	@Override
	public void updateInputs(AlgaerIOInputs input) {
		pivotSim.update(SIM_PERIOD);
		wheelMotorSim.update(SIM_PERIOD);

		input.wheelSpeed = Units.radiansToDegrees(wheelMotorSim.getAngularVelocityRadPerSec());
		input.wheelSpeedSetpoint = wheelSpeedSetpoint;
		input.pivotPosition = Units.radiansToDegrees(pivotSim.getAngleRads());
		input.pivotSetpoint = pivotPositionSetpoint;
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint.in(Degrees);
		pivotSim.setInputVoltage(
			pivotController.calculate(
				Units.radiansToDegrees(pivotSim.getAngleRads()),
				pivotSetpoint.in(Degrees)
			)
		);
	}

	@Override
	public void setWheelSpeed(AngularVelocity wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint.in(RotationsPerSecond);
		wheelMotorSim.setInputVoltage(
			wheelSpeedController.calculate(
				Units.radiansToRotations(wheelMotorSim.getAngularVelocityRadPerSec()),
				wheelSpeedSetpoint.in(RotationsPerSecond)
			)
		);
	}
}
