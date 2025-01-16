package frc.robot.Subsystems.Algaer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Algaer.AlgaerConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;

public class AlgaerIOReal implements AlgaerIO {

	private PIDController pivotController;
	private PIDController wheelSpeedController;
	private CANcoder absoluteEncoder;
	private double pivotPositionSetpoint;
	private double wheelSpeedSetpoint;

	private SparkMax pivotMotor;

	private SparkMax wheelMotor;
	private RelativeEncoder wheelEncoder;

	public AlgaerIOReal() {
		wheelMotor = new SparkMax(Real.WHEEL_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor = new SparkMax(Real.PIVOT_MOTOR_CANID, MotorType.kBrushless);
		absoluteEncoder = new CANcoder(Real.ABSOLUTE_ENCODER_CANID);

		wheelEncoder.setPosition(0);

		pivotController = PIVOT_CONTROLLER.get();
		wheelSpeedController = WHEEL_CONTROLLER.get();
	}

	@Override
	public void updateInputs(AlgaerIOInputs inputs) {
		inputs.pivotPosition = Units.rotationsToDegrees(absoluteEncoder.getPosition().getValue().in(Degree));
		inputs.pivotSetpoint = pivotPositionSetpoint;
		inputs.wheelSpeed = wheelEncoder.getVelocity() / 60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		if (GlobalConstants.ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Algaer Pivot PID", pivotController);
			SmartDashboard.putData("Algaer Wheel Speed PID", wheelSpeedController);
		}
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint.in(Degrees);
		double voltage = pivotController.calculate(Units.rotationsToDegrees(absoluteEncoder.getPosition().getValue().in(Degree)), pivotSetpoint.in(Degrees));
		pivotMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(AngularVelocity wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed.in(RotationsPerSecond);
		double voltage = wheelSpeedController.calculate(wheelEncoder.getVelocity() / 60, wheelSpeed.in(DegreesPerSecond));
		wheelMotor.setVoltage(voltage);
	}

	@Override
	public boolean nearTarget() {
		return (Math.abs(Units.rotationsToDegrees(absoluteEncoder.getPosition().getValue().in(Degree)) - pivotPositionSetpoint) < PIVOT_TOLERANCE.in(Degrees) && Math.abs(wheelEncoder.getVelocity() / 60 - wheelSpeedSetpoint) < WHEEL_TOLERANCE.in(RotationsPerSecond));
	}
}
