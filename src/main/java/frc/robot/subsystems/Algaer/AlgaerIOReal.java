package frc.robot.Subsystems.Algaer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Algaer.AlgaerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Subsystems.Algaer.AlgaerConstants.Real;

public class AlgaerIOReal implements AlgaerIO {

	private PIDController pivotController;
	private double pivotPositionSetpoint;
	private double wheelSpeedSetpoint;

	private SparkMax pivotMotor;
	private TalonFX wheelMotor;

	public AlgaerIOReal() {
		wheelMotor = new TalonFX(Real.WHEEL_MOTOR_CANID);
		pivotMotor = new SparkMax(Real.PIVOT_MOTOR_CANID, MotorType.kBrushless);

		pivotController = PIVOT_CONTROLLER.get();
		pivotMotor.getEncoder().setPosition(0);
		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData(SUBSYSTEM_NAME + "/Algaer Pivot PID", pivotController);
		}
	}

	@Override
	public void updateInputs(AlgaerIOInputs inputs) {
		inputs.pivotPosition = Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition() / OVERALL_GEARING);
		inputs.pivotSetpoint = pivotPositionSetpoint;
		inputs.wheelSpeed = wheelMotor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint.in(Degrees);
		double voltage = pivotController.calculate(Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition() / OVERALL_GEARING), pivotSetpoint.in(Degrees));
		pivotMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(double wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed;
		wheelMotor.set(wheelSpeed);
	}

	@Override
	public boolean nearTarget() {
		// System.out.println(Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition() / OVERALL_GEARING));
		// System.out.println(pivotPositionSetpoint);
		return Math.abs(Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition() / OVERALL_GEARING) - pivotPositionSetpoint) < PIVOT_TOLERANCE.in(Degrees);
	}

	@Override
	public Angle getAngle() {
		return Rotations.of(pivotMotor.getEncoder().getPosition() / OVERALL_GEARING);
	}

	@Override
	public TalonFX getWheelMotor() {
		return wheelMotor;
	}

	@Override
	public SparkMax getPivotMotor() {
		return pivotMotor;
	}
}
