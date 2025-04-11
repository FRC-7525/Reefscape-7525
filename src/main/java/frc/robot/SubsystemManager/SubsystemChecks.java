package frc.robot.SubsystemManager;

import static edu.wpi.first.units.Units.*;
import static frc.robot.SubsystemManager.SubsystemManagerStates.*;

import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Passthrough.Passthrough;

public class SubsystemChecks {
    private static SubsystemManager subsystemManager; 

    private SubsystemChecks() {
        subsystemManager = SubsystemManager.getInstance(); 
     }

    public static void run() {
        if ("Coraler".equals(System.getenv("CI_NAME"))) {
			subsystemManager.setState(SubsystemManagerStates.OUTTAKING);
			System.out.printf("%d %d %d\n", OUTTAKING.getCoralerState().getVelocitySupplier().get(), Coraler.getInstance().getMotor().getVelocity().getValueAsDouble(), subsystemManager.getTime());
		} else if ("Elevator".equals(System.getenv("CI_NAME"))) {
			subsystemManager.driverReefScoringLevel = 4;
			subsystemManager.setState(TRANSITIONING_SCORING_REEF);
			System.out.printf("%d %d %d\n", TRANSITIONING_SCORING_REEF.getElevatorStateSupplier().get().getTargetHeight().in(Meters), Elevator.getInstance().getCarriageHeight().in(Meters), subsystemManager.getTime());
		} else if ("Passthrough".equals(System.getenv("CI_NAME"))) {
			subsystemManager.setState(SubsystemManagerStates.INTAKING_CORALER);
			System.out.printf("%d %d %d\n", INTAKING_CORALER.getPassthroughState().getVelocity(), Passthrough.getInstance().getMotor().getVelocity().getValueAsDouble(), subsystemManager.getTime());
		}
    }
}
