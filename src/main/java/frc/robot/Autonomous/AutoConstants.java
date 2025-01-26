package frc.robot.Autonomous;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class AutoConstants {

	public static final Mass CHASSIS_MASS = Pounds.of(61.717);
	public static final Mass ELEVATOR_MASS = Pounds.of(34.445);
	public static final Mass TOTAL_MASS_LBS = ELEVATOR_MASS.plus(CHASSIS_MASS);


	public static final Distance CHASSIS_CG = Inches.of(6.22);
	public static final Distance BASE_ELEVATOR_CG = Inches.of(17.861);

	public static final Time TIPPING_CALCULATION_TIME = Seconds.of(0.5);

	public static Distance calculateVerticalCG(Distance elevatorHeight) {
		Distance elevatorCG = elevatorHeight.plus(BASE_ELEVATOR_CG);

		// Calculate the vertical center of gravity
		Distance zCG = Meters.of((CHASSIS_CG.times(CHASSIS_MASS).plus(elevatorCG.times(ELEVATOR_MASS))).div(TOTAL_MASS_LBS).magnitude());
		return zCG; 
	}
}
