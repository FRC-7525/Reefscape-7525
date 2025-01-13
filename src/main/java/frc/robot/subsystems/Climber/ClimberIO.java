package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {

		public double climberSpeed;

		public double climberPosition;

		public double climberAngularPosition;

		public double climberHeightPoint;
	}

	public void updateInputs(ClimberIOInputs inputs);

	public void zero();

	public boolean isZeroed();

	public boolean nearSetpoint();

	public void setSetpoint(Distance setpoint);

	public void stop();
}
