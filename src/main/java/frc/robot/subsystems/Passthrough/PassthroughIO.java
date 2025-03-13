package frc.robot.subsystems.Passthrough;

import org.littletonrobotics.junction.AutoLog;

public interface PassthroughIO {
	@AutoLog
	public class PassthroughIOInputs {

		double motorVelocityRPS;
		double inputVoltage;
		double targetVelocity;
	}

	public void updateInput(PassthroughIOInputs inputs);

	public void setTargetVelocity(double velocity);
}
