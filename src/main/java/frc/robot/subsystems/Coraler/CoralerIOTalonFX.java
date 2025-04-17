package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.SubsystemManager.SubsystemManagerConstants.L1_DEBOUNCE_TIME;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Subsystems.Coraler.CoralerConstants.Real;
import org.littletonrobotics.junction.Logger;

public class CoralerIOTalonFX implements CoralerIO {

	private final TalonFX velocityMotor;
	private final PIDController velocityController;
	// private final DigitalInput beamBreak;
	private double speedPoint;

	private Debouncer L1Debouncer = new Debouncer(L1_DEBOUNCE_TIME, DebounceType.kBoth);

	public CoralerIOTalonFX() {
		velocityMotor = new TalonFX(Real.WHEEL_MOTOR_CAN_ID);
		velocityMotor.setNeutralMode(NeutralModeValue.Brake);
		velocityMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(120)));
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
	public boolean gamepieceLeft() {
		boolean switchTripped = velocityMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
		Logger.recordOutput("Coraler/Switch Tripper", switchTripped);
		return L1Debouncer.calculate(!switchTripped);
	}

	@Override
	public boolean hasGamepiece() {
		// return !beamBreak.get();
		boolean switchTripped = velocityMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
		Logger.recordOutput("Coraler/Switch Tripper", switchTripped);
		return switchTripped;
	}
}
