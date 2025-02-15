package frc.robot.Subsystems.Passthrough;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class PassthroughConstants {

	public static final Current CURRENT_LIMIT = Amps.of(20);

	public static final Time BEAM_BREAK_DEBOUNCE_TIME = Milliseconds.of(5);

	public static final int VELOCITY_MOTOR_CAN_ID = 10;
	public static final int BEAM_BREAK_SENSOR_DIO_PORT = 0;

	public static final Time SIM_INTAKE_TIME = Seconds.of(0.5);

	public static final String SUBSYSTEM_NAME = "Passthrough";

	public static final boolean USE_CURRENT_SENSING = true;
}
