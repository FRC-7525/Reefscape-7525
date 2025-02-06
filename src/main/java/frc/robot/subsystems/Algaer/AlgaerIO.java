package frc.robot.Subsystems.Algaer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface AlgaerIO {
	@AutoLog
	public static class AlgaerIOInputs {

		// Pivot
		public double pivotPosition;
		public double pivotSetpoint;

		// Spinners
		public double wheelSpeed;
		public double wheelSpeedSetpoint;
	}

	public void updateInputs(AlgaerIOInputs input);

	public void setPivotSetpoint(Angle pivotSetpoint);

	public void setWheelSpeed(AngularVelocity wheelSpeed);

	public Angle getAngle();

	public boolean nearTarget();

	public TalonFX getWheelMotor();

	public TalonFX getPivotMotor();
}
