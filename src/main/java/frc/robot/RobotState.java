package frc.robot;

import frc.robot.Subsystems.AutoAlign.AutoAlign;
import frc.robot.Subsystems.AutoAlign.AATypeManager.AATypeManager;
import frc.robot.Subsystems.Coraler.Coraler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.Passthrough.Passthrough;
import frc.robot.Subsystems.Vision.Vision;

import static frc.robot.Subsystems.Vision.VisionConstants.*;
import static frc.robot.GlobalConstants.*;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
    // Big static class of all the subsystems, less messy than just using singleton from all of them
    // and more dynamic than dependency injection (which we should probably use)

    private static RobotState instance;

    private static SubsystemProvider<?> driveProvider, ledProvider, coralerProvider, elevatorProvider, 
        autoAlignProvider, passthroughProvider, frontVisionProvider, backVisionProvider, aaTypeManager;

    @FunctionalInterface
    public interface SubsystemProvider<T> {
        T get();
    }

    private RobotState() {
        driveProvider = Drive::getInstance;
        ledProvider = LED::getInstance;
        coralerProvider = Coraler::getInstance;
        elevatorProvider = Elevator::getInstance;
        autoAlignProvider = AutoAlign::getInstance;
        passthroughProvider = Passthrough::getInstance;
        aaTypeManager = AATypeManager::getInstance;
        frontVisionProvider = () -> new Vision("Front Vision", visionMeasurment -> System.out.println("TODO: steal 6328"), ROBOT_MODE == RobotMode.REAL ? FRONT_REAL_IOS : FRONT_SIM_IOS);
        backVisionProvider = () -> new Vision("Back Vision",  visionMeasurment -> getDrive().addVisionMeasurement(visionMeasurment.pose(), visionMeasurment.timestamp(), visionMeasurment.standardDev()), ROBOT_MODE == RobotMode.REAL ? BACK_REAL_IOS : BACK_SIM_IOS);
    }


    public RobotState getState() {
        if (instance
         == null) {
            instance = new RobotState();
        }
        return this;
    }

    // if u want to access subsystems directly
   public static Drive getDrive() {
       return (Drive) driveProvider.get();
   }

    public static LED getLED() {
         return (LED) ledProvider.get();
    }

    public static Coraler getCoraler() {
        return (Coraler) coralerProvider.get();
    }

    public static Elevator getElevator() {
        return (Elevator) elevatorProvider.get();
    }

    public static AutoAlign getAutoAlign() {
        return (AutoAlign) autoAlignProvider.get();
    }

    public static Passthrough getPassthrough() {
        return (Passthrough) passthroughProvider.get();
    }

    public static Vision getFrontVision() {
        return (Vision) frontVisionProvider.get();
    }

    public static Vision getBackVision() {
        return (Vision) backVisionProvider.get();
    }

    public static AATypeManager getAATypeManager() {
        return (AATypeManager) aaTypeManager.get();
    }

   // General util, less messy
   public static Pose2d getPose() {
       return getDrive().getPose();
   }
    
}
