package frc.robot.Subsystems.Passthrough;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Passthrough.PassthroughConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PassthroughIOSim implements PassthroughIO {

	final DCMotorSim motorSim;
	final TalonFX motor;
	final TalonFXSimState motorSimState;
	double targetVelocity;

	public PassthroughIOSim() {
		motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(Sim.NUM_MOTORS), Sim.MOTOR_MOI, GEARING), DCMotor.getFalcon500(1));
		motor = new TalonFX(WHEEL_MOTOR_CAN_ID);
		motor.setNeutralMode(NeutralModeValue.Brake);
		motorSimState = new TalonFXSimState(motor);

		targetVelocity = 0;
	}

	@Override
	public void updateInput(PassthroughIOInputs inputs) {
		inputs.inputVoltage = motorSim.getInputVoltage();
		inputs.motorVelocityRPS = motorSim.getAngularVelocity().in(RotationsPerSecond);
		inputs.targetVelocity = targetVelocity;

		motorSimState.setSupplyVoltage(inputs.inputVoltage);
		motorSimState.setRotorVelocity(motorSim.getAngularVelocity());
	}

	@Override
	public void setTargetVelocity(double targetVelocity) {
		this.targetVelocity = targetVelocity;
		motorSim.setInputVoltage(targetVelocity * 12);
	}
}
