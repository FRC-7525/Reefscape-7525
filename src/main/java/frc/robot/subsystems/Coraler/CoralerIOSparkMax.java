package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

public class CoralerIOSparkMax implements CoralerIO {

	private final SparkMax velocityMotor;
	private final RelativeEncoder velocityEncoder;
	private final PIDController wheelController;
	private double speedPoint;

	public CoralerIOSparkMax() {
		velocityMotor = new SparkMax(Real.WHEEL_MOTOR_CAN_ID, MotorType.kBrushless);
		velocityEncoder = velocityMotor.getEncoder();
		wheelController = WHEEL_CONTROLLER.get();
		speedPoint = 0.0;
	}

	@Override
	public void updateInputs(CoralerIOInputs inputs) {
		inputs.velocityRPS = velocityEncoder.getVelocity();
		inputs.speedPointRPS = speedPoint;
	}

	@Override
	public void setVelocity(AngularVelocity speedPoint) {
		this.speedPoint = speedPoint.in(RotationsPerSecond);
		velocityMotor.setVoltage(
			wheelController.calculate(
				velocityEncoder.getVelocity(),
				speedPoint.in(RotationsPerSecond)
			)
		);
	}
}
