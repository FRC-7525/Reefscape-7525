package frc.robot.Subsystems.LED;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class LEDIOSim implements LEDIO {

	private final PWM led = new PWM(0);
	private final PWMSim ledSim = new PWMSim(led);
	private int setSpeed = -99;

	@Override
	public void setPWMState(int signal) {
		led.setPulseTimeMicroseconds(signal);
		ledSim.setPulseMicrosecond(signal);
		setSpeed = signal;
	}

	@Override
	public void updateInputs(LEDIOInputs inputs) {
		inputs.pwmSpeed = led.getPulseTimeMicroseconds();
		inputs.pwmSetSpeed = setSpeed;
	}
}
