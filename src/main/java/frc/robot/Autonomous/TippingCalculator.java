package frc.robot.Autonomous;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
public class TippingCalculator {

    private Mass robotMass; // Mass of the robot
    private Distance cgHeight; // Height of the CG from the ground
    private Distance cgDistance; // Horizontal distance from CG to tipping axis

    /**
     * @param mass      Mass of the robot
     * @param wheelbase Wheelbase length
     */
    public TippingCalculator(Mass mass, Distance wheelbase) {
        this.robotMass = mass;
        // TODO: Care if or not the CG is at the center
        this.cgDistance = wheelbase.div(2); // Assume CG is at the center, idrc if its not
        // No division errors pls
        cgHeight = Meters.of(0.001);
    }

    /**
     * Updates the height of the CG based on dynamic changes (we got an elevator).
     * 
     * @param cgHeight
     */
    public void updateCGHeight(Distance cgHeight) {
        this.cgHeight = cgHeight;
    }

    /**
     * Updates the horizontal distance of the CG from the tipping axis.
     * 
     * @param cgDistance Horizontal distance from CG to tipping axis
     */
    public void updateCGDistance(Distance cgDistance) {
        this.cgDistance = cgDistance;
    }

    /**
     * Calculates the deceleration required to stop in a given time.
     * 
     * @param velocity Current velocity of the robot
     * @param time     Time to stop
     * @return Deceleration
     */
    public LinearAcceleration calculateDeceleration(LinearVelocity velocity, Time time) {
        return velocity.div(time);
    }

    /**
     * Calculates the minimum average deceleration required to stop the robot.
     * 
     * @param endingPose  The target pose where the robot needs to stop
     * @param currentPose The current pose of the robot
     * @param velocity    The current velocity of the robot
     * @return The minimum average deceleration required to stop
     */
    public LinearAcceleration calculateDeceleration(Pose2d endingPose, Pose2d currentPose, LinearVelocity velocity) {
        double distance = currentPose.getTranslation().getDistance(endingPose.getTranslation());

        if (distance == 0) {
            return MetersPerSecondPerSecond.of(0);
        }

        double initialVelocity = velocity.in(MetersPerSecond);
        double deceleration = -(initialVelocity * initialVelocity) / (2 * distance);
        return MetersPerSecondPerSecond.of(deceleration);
    }

    /**
     * Calculates the tipping torque due to deceleration.
     * 
     * @param deceleration Deceleration rate
     * @return Tipping torque
     */
    public Torque calculateTippingTorque(LinearAcceleration deceleration) {
        Force horizontalForce = robotMass.times(deceleration);
        return NewtonMeters.of(horizontalForce.in(Newtons) * cgHeight.in(Meters));
    }

	/**
	 * Calculates the restoring torque due to gravity.
	 *
	 * @return Restoring torque
	 */
	public Torque calculateRestoringTorque() {
		return NewtonMeters.of(robotMass.times(GRAVITY).in(Newtons) * cgDistance.in(Meters));
	}

    /**
     * Determines if the robot will tip based on velocity, deceleration, and CG.
     * 
     * @param velocity Current velocity of the robot
     * @param time     Time to stop
     * @return True if the robot will tip, false otherwise.
     */
    public boolean willTip(LinearVelocity velocity, Time time) {
        LinearAcceleration deceleration = calculateDeceleration(velocity, time);
        Torque tippingTorque = calculateTippingTorque(deceleration);
        Torque restoringTorque = calculateRestoringTorque();

        return tippingTorque.gt(restoringTorque);
    }

    /**
     * Determines if the robot will tip based on velocity, deceleration, and CG.
     * 
     * @param velocity Current velocity of the robot
     * @param time     Time to stop
     * @return True if the robot will tip, false otherwise.
     */
    public boolean willTip(Pose2d endingPose, Pose2d currentPose, LinearVelocity velocity) {
        LinearAcceleration deceleration = calculateDeceleration(endingPose, currentPose, velocity);
        Torque tippingTorque = calculateTippingTorque(deceleration);
        Torque restoringTorque = calculateRestoringTorque();

        return tippingTorque.gt(restoringTorque);
    }
}