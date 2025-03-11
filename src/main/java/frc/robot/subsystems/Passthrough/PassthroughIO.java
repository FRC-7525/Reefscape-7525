package frc.robot.Subsystems.Passthrough;

public interface PassthroughIO {
    public static class PassthroughIOInputs {
        double motorVelocityRPS;
        double inputVoltage;
        double targetVelocity;
    }

    public void updateInput(PassthroughIOInputs inputs);
    
    public void setTargetVelocity(double velocity);
}
