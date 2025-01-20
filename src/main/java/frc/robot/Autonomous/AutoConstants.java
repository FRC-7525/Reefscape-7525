package frc.robot.Autonomous;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class AutoConstants {
    public static final Mass TOTAL_MASS_LBS = Pounds.of(110.0); 
    public static final Mass CHASSIS_MASS = Pounds.of(60.0); 
    public static final Mass ELEVATOR_MASS = Pounds.of(30.0); 
    public static final Mass CARRIAGE_MASS = Pounds.of(17.0); 

    public static final Distance CHASSIS_HEIGHT_IN = Inches.of(10.0);
    public static final Distance CARRIAGE_SIZE = Inches.of(5.0);
    public static final Distance CARRAIGE_ELEVATOR_OFFSET = Inches.of(4);

    public static final Time TIPPING_CALCULATION_TIME = Seconds.of(0.5);

    public static Distance calculateVerticalCG(Distance elevatorHeight, Distance carriageHeight) {
        Distance chassisCG = CHASSIS_HEIGHT_IN.div(2); 
        Distance elevatorCG = elevatorHeight.div(2);
        Distance carriageCG = carriageHeight.div(2); 

        // Calculate the vertical CG
        Distance zCG = Meters.of(CHASSIS_MASS.in(Kilograms) * chassisCG.in(Meters) + 
            ELEVATOR_MASS.in(Kilograms) * elevatorCG.in(Meters) + 
            CARRIAGE_MASS.in(Kilograms) * carriageCG.in(Meters)).div(TOTAL_MASS_LBS.in(Kilograms));

        return zCG; // Return the vertical CG
    }
}
