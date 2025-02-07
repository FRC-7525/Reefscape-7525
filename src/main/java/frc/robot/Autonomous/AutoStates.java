package frc.robot.Autonomous;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** This enum is meant for storing robot states in auto. We still have to use commands but its 
 * a better way to follow our state machine design pattern while maintaining readability
 * and not using half command based half state based design :Skull:.
*/

public enum AutoStates {
    IDLE("IDLE", () -> AutoBuilder.followPath(PathPlannerPath.fromPathFile("zz")), () -> new InstantCommand()),
    CIL("Coral Intake Left Station", () -> AutoBuilder.followPath(PathPlannerPath.fromPathFile("LALA")), () -> AutoCommands.IntakeCoral.getCoral()),
    CIR("Coral Intake Right Station", () -> AutoBuilder.followPath(PathPlannerPath.fromPathFile("Lalal")), () -> AutoCommands.IntakeCoral.getCoral()),
    C("Score Coral", () -> AutoBuilder.followPath(PathPlannerPath.fromPathFile("lz")), new InstantCommand()),;


    public String stateString;
    public Supplier<Command> associatedPath;
    public Supplier<Command> associatedAction;

    AutoStates(String stateString, Supplier<Command> associatedPath, Supplier<Command> associatedAction) {
        this.stateString = stateString;
        this.associatedPath = associatedPath;
        this.associatedAction = associatedAction;

    }

    public String getStateString() {
        return stateString;
    }

    public Supplier<Command> getAssociatedPath() {
        return associatedPath;
    }

    public Supplier<Command> getAssociatedAction() {
        return associatedAction;
    }
}
