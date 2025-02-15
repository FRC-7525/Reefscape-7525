package frc.robot.Subsystems.Passthrough;

import org.team7525.subsystem.SubsystemStates;

public enum PassthroughStates implements SubsystemStates {

    INTAKE(0.5, "Intake"),
    IDLE(0.0, "Idle"),
    FEEDING(0.2, "Feeding"),;

    PassthroughStates(double speedPoint, String stateString) {
        this.speedPoint = speedPoint;
        this.stateString = stateString;
    }

    String stateString;
    double speedPoint;

    public double getSpeedPoint() {
        return speedPoint;
    }

    @Override
    public String getStateString() {
        return stateString;
    }
}
