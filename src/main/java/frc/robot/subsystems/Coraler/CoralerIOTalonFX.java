package frc.robot.subsystems.Coraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Coraler.CoralerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.subsystems.Coraler.CoralerConstants.Real;
import org.littletonrobotics.junction.Logger;

public class CoralerIOTalonFX implements CoralerIO {

	private final TalonFX velocityMotor;
	private final PIDController velocityController;
	// private final DigitalInput beamBreak;
	private double speedPoint;

	public CoralerIOTalonFX() {
		velocityMotor = new TalonFX(Real.WHEEL_MOTOR_CAN_ID);
		velocityMotor.setNeutralMode(NeutralModeValue.Brake);
		// beamBreak = new DigitalInput(Real.DIO_PORT);

		velocityController = VELOCITY_CONTROLLER.get();
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
	public void setVelocity(double speedPoint) {
		velocityMotor.set(speedPoint);
	}

	@Override
	public TalonFX getMotor() {
		return velocityMotor;
	}

	@Override
	public boolean currentLimitReached() {
		return velocityMotor.getStatorCurrent().getValue().in(Amp) > STATOR_CURRENT_SENSING_LIMIT;
	}

	@Override
	public boolean hasGamepiece() {
		// return !beamBreak.get();
		boolean switchTripped = velocityMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
		Logger.recordOutput("Coraler/Switch Tripper", switchTripped);
		return switchTripped;
	}
}
