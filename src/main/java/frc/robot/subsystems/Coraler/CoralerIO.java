package frc.robot.Subsystems.Coraler;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface CoralerIO {
	@AutoLog
	class CoralerIOInputs {

		public double velocityRPS;
		public double speedPointRPS;
	}

	public void updateInputs(CoralerIOInputs inputs);

	public void setVelocity(double speedPoint);

	public boolean currentLimitReached();

	public boolean hasGamepiece();

	public TalonFX getMotor();
}
