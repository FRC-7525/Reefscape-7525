package frc.robot.Subsystems.Algaer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Algaer.AlgaerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Subsystems.Algaer.AlgaerConstants.Real;

public class AlgaerIOReal implements AlgaerIO {

	private PIDController pivotController;
	private PIDController wheelSpeedController;
	private DutyCycleEncoder absoluteEncoder;
	private double pivotPositionSetpoint;
	private double wheelSpeedSetpoint;

	private TalonFX pivotMotor;
	private TalonFX wheelMotor;

	public AlgaerIOReal() {
		wheelMotor = new TalonFX(Real.WHEEL_MOTOR_CANID);
		pivotMotor = new TalonFX(Real.PIVOT_MOTOR_CANID);
		absoluteEncoder = new DutyCycleEncoder(Real.ABSOLUTE_ENCODER_PORT);

		pivotController = PIVOT_CONTROLLER.get();
		wheelSpeedController = WHEEL_CONTROLLER.get();
		pivotMotor.setPosition(absoluteEncoder.get()/ABSOLUTE_ENCODER_GEARING - ABSOLUTE_ENCODER_OFFSET.in(Rotations));
		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData(SUBSYSTEM_NAME + "/Algaer Pivot PID", pivotController);
			SmartDashboard.putData(SUBSYSTEM_NAME + "/Algaer Wheel Speed PID", wheelSpeedController);
		}
	}

	@Override
	public void updateInputs(AlgaerIOInputs inputs) {
		inputs.pivotPosition = Units.rotationsToDegrees(pivotMotor.getPosition().getValue().in(Degree)/OVERALL_GEARING);
		inputs.pivotSetpoint = pivotPositionSetpoint;
		inputs.wheelSpeed = wheelMotor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint.in(Degrees);
		double voltage = pivotController.calculate(pivotMotor.getPosition().getValue().in(Degree)/OVERALL_GEARING, pivotSetpoint.in(Degrees));
		pivotMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(AngularVelocity wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed.in(RotationsPerSecond);
		double voltage = wheelSpeedController.calculate(wheelMotor.getVelocity().getValue().in(RotationsPerSecond), wheelSpeed.in(DegreesPerSecond));
		wheelMotor.setVoltage(voltage);
	}

	@Override
	public boolean nearTarget() {
		return (Math.abs(pivotMotor.getPosition().getValue().in(Degree) - pivotPositionSetpoint) < PIVOT_TOLERANCE.in(Degrees) && Math.abs(wheelMotor.getVelocity().getValue().in(RotationsPerSecond) - wheelSpeedSetpoint) < WHEEL_TOLERANCE.in(RotationsPerSecond));
	}

	@Override
	public Angle getAngle() {
		return Rotations.of(pivotMotor.getPosition().getValue().in(Degree)/OVERALL_GEARING);
	}
}
