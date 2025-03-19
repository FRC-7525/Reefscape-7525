package frc.robot.Subsystems.AutoAlign.AATypeManager;

import org.team7525.subsystem.SubsystemStates;

import frc.robot.Subsystems.AutoAlign.AutoAlign;

public enum AATypeManagerStates implements SubsystemStates {
    OFF("Off", () -> {}),
    REPULSOR("Repulsor AA", () -> AutoAlign.getInstance().executeRepulsorAutoAlign()),
    REGULAR("Regular AA", () -> AutoAlign.getInstance().executeScaledFeedforwardAutoAlign());

    String stateString;
    Runnable autoAlignMethod;

    AATypeManagerStates(String stateString, Runnable autoAlignMethod) {
        this.stateString = stateString;
        this.autoAlignMethod = autoAlignMethod;
    }

    @Override
    public String getStateString() {
        return stateString;
    }

    public void runAlignmentRunnable() {
        autoAlignMethod.run();
    }
}
