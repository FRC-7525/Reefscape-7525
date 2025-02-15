package frc.robot.Subsystems.Passthrough;

import static frc.robot.Subsystems.Passthrough.PassthroughConstants.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Coraler.CoralerConstants.Sim;

public class PassthroughIOSim implements PassthroughIO {
	private DCMotorSim motorSim;
	private TalonFXSimState motorSimState;
	private TalonFX velocityMotor;
	private double speedPoint;

	public PassthroughIOSim() {
		motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.0001, 1), DCMotor.getNEO(1));
		velocityMotor = new TalonFX(VELOCITY_MOTOR_CAN_ID);
		motorSimState = new TalonFXSimState(velocityMotor);
		speedPoint = 0.0;
	}

	@Override
	public void updateInputs(PassthroughIOInputs inputs) {
		inputs.velocityRPS = velocityMotor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.speedPoint = speedPoint;

		motorSim.update(GlobalConstants.SIMULATION_PERIOD);
		motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60);
		motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations());
	}

	@Override
	public void setVelocity(double speedPoint) {
		motorSim.setInputVoltage(speedPoint * 12);
        this.speedPoint = speedPoint;
	}

    @Override
	public boolean hasGamepiece() {
		return Coraler.getInstance().getStateTime() > Sim.INTAKE_TIME.in(Seconds);
	}

	@Override
	public boolean currentLimitReached() {
		return Coraler.getInstance().getStateTime() > SIM_INTAKE_TIME.in(Seconds);
	}
}
