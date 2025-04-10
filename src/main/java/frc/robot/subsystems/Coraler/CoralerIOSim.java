package frc.robot.Subsystems.Coraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Coraler.CoralerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.Coraler.CoralerConstants.Sim;

public class CoralerIOSim implements CoralerIO {

	private DCMotorSim motorSim;
	private TalonFXSimState motorSimState;
	private TalonFX velocityMotor;
	private double speedPoint;

	public CoralerIOSim() {
		motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(Sim.NUM_MOTORS), Sim.MOTOR_MOI, GEARING), DCMotor.getNEO(Sim.NUM_MOTORS));
		velocityMotor = new TalonFX(Real.WHEEL_MOTOR_CAN_ID);
		motorSimState = new TalonFXSimState(velocityMotor);
		speedPoint = 0.0;
	}

	@Override
	public void updateInputs(CoralerIOInputs inputs) {
		inputs.velocityRPS = velocityMotor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.speedPointRPS = speedPoint;

		motorSim.update(GlobalConstants.SIMULATION_PERIOD);
		motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60);
		motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations());
	}

	@Override
	public void setVelocity(double speedPoint) {
		motorSim.setInputVoltage(speedPoint * SET_TO_VOLTS_CF);
	}

	@Override
	public TalonFX getMotor() {
		return velocityMotor;
	}

	@Override
	public boolean currentLimitReached() {
		return hasGamepiece();
	}

	@Override
	public boolean hasGamepiece() {
		return Coraler.getInstance().getStateTime() > Sim.INTAKE_TIME.in(Seconds) && AutoAlign.getInstance().nearGoal();
	}
}
