package frc.robot.Subsystems.Passthrough;

import org.littletonrobotics.junction.AutoLog;

public interface PassthroughIO {
	@AutoLog
	public class PassthroughIOInputs {

		public double velocityRPS;
		public double speedPoint;
		public boolean hasGamePiece;
	}

	public void updateInputs(PassthroughIOInputs inputs);

	public void setVelocity(double speedPoint);

	public boolean currentLimitReached();

	public boolean hasGamepiece();
}
