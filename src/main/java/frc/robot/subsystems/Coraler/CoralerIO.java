package frc.robot.Subsystems.Coraler;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface CoralerIO {
	@AutoLog
	class CoralerIOInputs {

		public double velocityRPS;
		public double speedPointRPS;
	}

	public void updateInputs(CoralerIOInputs inputs);

	public void setVelocity(AngularVelocity speedPoint);

	public boolean firstDetectorTripped();

	public boolean secondDetectorTripped();

	public TalonFX getMotor();
}
