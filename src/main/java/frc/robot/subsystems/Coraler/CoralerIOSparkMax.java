package frc.robot.subsystems.Coraler;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.Coraler.CoralerConstants.*;

public class CoralerIOSparkMax implements CoralerIO {

    private final SparkMax velocityMotor;
    private final RelativeEncoder velocityEncoder;
    private final PIDController velocityController;
    private double speedPoint;

    public CoralerIOSparkMax() {
        velocityMotor = new SparkMax(VELOCITY_MOTOR_CAN_ID, MotorType.kBrushless);
        velocityEncoder = velocityMotor.getEncoder();
        velocityController = new PIDController(Real.VELOCITY_PID.kP, Real.VELOCITY_PID.kI, Real.VELOCITY_PID.kD);
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
        velocityMotor.setVoltage(velocityController.calculate(velocityEncoder.getVelocity(), speedPoint.in(RotationsPerSecond)));
    }
}
