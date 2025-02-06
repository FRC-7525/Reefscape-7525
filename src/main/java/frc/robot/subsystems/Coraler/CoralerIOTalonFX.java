package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralerIOTalonFX implements CoralerIO {

	private final TalonFX velocityMotor;
	private final PIDController velocityController;
	private final DigitalInput gamePieceInitialIntakeDetector;
	private final DigitalInput gamepieceCenteringDetector;
	private double speedPoint;

	public CoralerIOTalonFX() {
		velocityMotor = new TalonFX(Real.WHEEL_MOTOR_CAN_ID);
		velocityController = VELOCITY_CONTROLLER.get();
		gamePieceInitialIntakeDetector = new DigitalInput(Real.GAMEPIECE_INITIAL_DETECTOR_DIO_PORT);
		gamepieceCenteringDetector = new DigitalInput(Real.GAMEPIECE_CENTERING_DETECTOR_DIO_PORT);
		speedPoint = 0.0;

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Coraler Wheel PID", velocityController);
		}
	}

	@Override
	public void updateInputs(CoralerIOInputs inputs) {
		inputs.velocityRPS = velocityMotor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.speedPointRPS = speedPoint;
	}

	@Override
	public void setVelocity(AngularVelocity speedPoint) {
		this.speedPoint = speedPoint.in(RotationsPerSecond);
		velocityMotor.setVoltage(velocityController.calculate(velocityMotor.getVelocity().getValue().in(RotationsPerSecond), speedPoint.in(RotationsPerSecond)));
	}

	@Override
	public boolean firstDetectorTripped() {
		return gamePieceInitialIntakeDetector.get();
	}

	@Override
	public boolean secondDetectorTripped() {
		return gamepieceCenteringDetector.get();
	}

	@Override
	public TalonFX getMotor() {
		return velocityMotor;
	}
}
