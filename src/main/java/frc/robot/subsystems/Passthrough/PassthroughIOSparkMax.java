package frc.robot.Subsystems.Passthrough;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Subsystems.Passthrough.PassthroughConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;

public class PassthroughIOSparkMax implements PassthroughIO {

	private SparkMax velocityMotor;
	private DigitalInput beamBreakSensor;
	private Debouncer beamBreakDebouncer;
	private double speedPoint;

	public PassthroughIOSparkMax() {
		velocityMotor = new SparkMax(VELOCITY_MOTOR_CAN_ID, MotorType.kBrushless);
		beamBreakDebouncer = new Debouncer(BEAM_BREAK_DEBOUNCE_TIME.in(Seconds), DebounceType.kBoth);
		beamBreakSensor = new DigitalInput(BEAM_BREAK_SENSOR_DIO_PORT);
		speedPoint = 0.0;
	}

	@Override
	public void updateInputs(PassthroughIOInputs inputs) {
		inputs.velocityRPS = velocityMotor.getEncoder().getVelocity();
		inputs.speedPoint = speedPoint;
		inputs.hasGamePiece = beamBreakDebouncer.calculate(beamBreakSensor.get());
	}

	@Override
	public void setVelocity(double speedPoint) {
		velocityMotor.set(speedPoint);
		this.speedPoint = speedPoint;
	}

	@Override
	public boolean currentLimitReached() {
		return velocityMotor.getOutputCurrent() > CURRENT_LIMIT.in(Amp);
	}

	@Override
	public boolean hasGamepiece() {
		return beamBreakDebouncer.calculate(beamBreakSensor.get());
	}
}
