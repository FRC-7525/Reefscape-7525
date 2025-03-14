package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
	@AutoLog
	public static class ElevatorIOInputs {

		public String stateString;
		public double currentElevatorHeight;
		public double elevatorHeightSetpoint;
		public double elevatorHeightGoalpoint;
		public double elevatorVelocity;
		public double elevatorVelocitySetpoint;
		public double elevatorVelocityGoalpoint;
		public double leftMotorVoltInput;
		public double rightMotorVoltInput;
		public boolean elevatorZeroed;
	}

	public void setHeightGoalpoint(Distance height);

	public void updateInputs(ElevatorIOInputs inputs);

	public void runElevator();

	public void zero();

	public void resetMotorsZeroed();

	public boolean motorsZeroed();

	public boolean nearTarget();

	public Distance getHeight();

	public Distance getStageOneHeight();

	public Distance getCarriageHeight();

	public TalonFX getLeftMotor();

	public TalonFX getRightMotor();

	public void resetController();
}
