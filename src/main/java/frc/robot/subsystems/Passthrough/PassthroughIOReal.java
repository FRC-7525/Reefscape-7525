package frc.robot.subsystems.Passthrough;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.Passthrough.PassthroughConstants.WHEEL_MOTOR_CAN_ID;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PassthroughIOReal implements PassthroughIO {

	final TalonFX motor;
	double targetVelocity;

	public PassthroughIOReal() {
		motor = new TalonFX(WHEEL_MOTOR_CAN_ID);
		motor.setNeutralMode(NeutralModeValue.Brake);
		targetVelocity = 0;
	}

	@Override
	public void updateInput(PassthroughIOInputs inputs) {
		inputs.inputVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.motorVelocityRPS = motor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.targetVelocity = targetVelocity;
	}

	@Override
	public void setTargetVelocity(double targetVelocity) {
		this.targetVelocity = targetVelocity;
		motor.set(targetVelocity);
	}
}
