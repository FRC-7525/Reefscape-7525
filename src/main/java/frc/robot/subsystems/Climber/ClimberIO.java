package frc.robot.Subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {

		public double climberSpeed;

		public double climberPosition;

		public double climberAngularPosition;

		public double climberHeightPoint;
	}

	public void updateInputs(ClimberIOInputs inputs);

	public boolean nearSetpoint();

	public Distance getPosition();

	public void setSetpoint(Distance setpoint);

	public void stop();
}
