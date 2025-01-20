package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

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

	public void setHeightGoalpoint(double height);

	public void updateInputs(ElevatorIOInputs inputs);

	public void runElevator();

	public void zero();

	public boolean isZeroed();

	public boolean nearTarget();

	public Distance getHeight();
}
