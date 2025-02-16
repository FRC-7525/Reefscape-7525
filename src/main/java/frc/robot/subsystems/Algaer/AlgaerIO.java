package frc.robot.Subsystems.Algaer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

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

	public void setWheelSpeed(double wheelSpeed);

	public Angle getAngle();

	public boolean nearTarget();

	public TalonFX getWheelMotor();

	public SparkMax getPivotMotor();
}
