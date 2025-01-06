package frc.robot.subsystems.Coraler;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface CoralerIO {
    
    @AutoLog
    class CoralerIOInputs {
        public double velocityRPS;
        public double speedPointRPS;
    }

    public default void updateInputs(CoralerIOInputs inputs) {};

    public default void setVelocity(AngularVelocity speedPoint) {};
}
