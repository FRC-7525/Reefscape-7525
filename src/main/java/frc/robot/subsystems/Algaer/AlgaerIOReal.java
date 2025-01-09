package frc.robot.subsystems.Algaer;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;

public class AlgaerIOReal implements AlgaerIO {

	private PIDController pivotController;
	private PIDController wheelSpeedController;
	private double pivotPositionSetpoint;
	private double wheelSpeedSetpoint;

	private SparkMax pivotMotor;
	private RelativeEncoder pivotEncoder;

	private SparkMax wheelMotor;
	private RelativeEncoder wheelEncoder;

	public AlgaerIOReal() {
		wheelMotor = new SparkMax(AlgaerConstants.Real.WHEEL_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor = new SparkMax(AlgaerConstants.Real.PIVOT_MOTOR_CANID, MotorType.kBrushless);
		pivotEncoder = pivotMotor.getEncoder();

		wheelEncoder.setPosition(0);
		pivotEncoder.setPosition(0);

		pivotController = new PIDController(
			AlgaerConstants.Real.PIVOT_PID_CONSTANTS.kP,
			AlgaerConstants.Real.PIVOT_PID_CONSTANTS.kI,
			AlgaerConstants.Real.PIVOT_PID_CONSTANTS.kD
		);
		wheelSpeedController = new PIDController(
			AlgaerConstants.Real.WHEEL_PID_CONSTANTS.kP,
			AlgaerConstants.Real.WHEEL_PID_CONSTANTS.kI,
			AlgaerConstants.Real.WHEEL_PID_CONSTANTS.kD
		);
	}

	@Override
	public void updateInputs(AlgaerIOInputs inputs) {
		inputs.pivotPosition = Units.rotationsToDegrees(pivotEncoder.getPosition());
		inputs.pivotSetpoint = pivotPositionSetpoint;
		inputs.wheelSpeed = wheelEncoder.getVelocity()/60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		if (GlobalConstants.ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Algaer Pivot PID", pivotController);
			SmartDashboard.putData("Algaer Wheel Speed PID", wheelSpeedController);
		}
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint.in(Degrees);
		double voltage = pivotController.calculate(
			Units.rotationsToDegrees(pivotEncoder.getPosition()),
			pivotSetpoint.in(Degrees)
		);
		pivotMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(AngularVelocity wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed.in(RotationsPerSecond);
		double voltage = wheelSpeedController.calculate(
			wheelEncoder.getVelocity()/60,
			wheelSpeed.in(DegreesPerSecond)
		);
		wheelMotor.setVoltage(voltage);
	}
}
