package frc.robot.Subsystems.Coraler;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface CoralerIO {
	@AutoLog
	class CoralerIOInputs {

		public double velocityRPS;
		public double speedPointRPS;
	}

	public default void updateInputs(CoralerIOInputs inputs) {}

	public default void setVelocity(AngularVelocity speedPoint) {}

}
