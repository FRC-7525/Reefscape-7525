package frc.robot.Subsystems.Passthrough;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface PassthroughIO {
	@AutoLog
	public class PassthroughIOInputs {

		double motorVelocityRPS;
		double inputVoltage;
		double targetVelocity;
	}

	public void updateInput(PassthroughIOInputs inputs);

	public void setTargetVelocity(double velocity);

	public TalonFX getMotor();
}
