package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.L1_HEIGHT;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.L1_SCORING_HEIGHT;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum ElevatorStates implements SubsystemStates {
	TRANSITIONING("Transitioning Elevator Level", () -> ElevatorConstants.TRANSITION_HEIGHT),
	L4("L4", () -> ElevatorConstants.L4_HEIGHT),
	L3("L3", () -> ElevatorConstants.L3_HEIGHT),
	L2("L2", () -> ElevatorConstants.L2_HEIGHT),
	L1("L1", () -> Inches.of(SmartDashboard.getNumber("L1 Height", L1_HEIGHT.in(Inches)))),
	CORAL_STATION("Coral Station", () -> ElevatorConstants.L1_HEIGHT),
	ZEROING("Zeroing", () -> ElevatorConstants.IDLE_HEIGHT),
	IDLE("Idle", () -> ElevatorConstants.IDLE_HEIGHT),
	L1_SCORING("L1 Scoring", () -> Inches.of(SmartDashboard.getNumber("L1 Scoring Height", L1_SCORING_HEIGHT.in(Inches))));

	//TODO: REVERT OFF OF SUPPLIERS WHEN DONE
	ElevatorStates(String stateString, Supplier<Distance> targetHeight) {
		this.targetHeight = targetHeight.get();
		this.stateString = stateString;
	}

	private Distance targetHeight;
	private String stateString;

	public Distance getTargetHeight() {
		return targetHeight;
	}

	public String getStateString() {
		return stateString;
	}
}
