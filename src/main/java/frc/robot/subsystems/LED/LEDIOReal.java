package frc.robot.subsystems.LED;

import static frc.robot.subsystems.LED.LEDConstants.*;

import edu.wpi.first.wpilibj.PWM;

public class LEDIOReal implements LEDIO {

	private final PWM led = new PWM(LED_PWM_PORT);
	private int setSpeed = -99;

	@Override
	public void setPWMState(int signal) {
		led.setPulseTimeMicroseconds(signal);
		setSpeed = signal;
	}

	@Override
	public void updateInputs(LEDIOInputs inputs) {
		inputs.pwmSpeed = led.getPulseTimeMicroseconds();
		inputs.pwmSetSpeed = setSpeed;
	}
}
