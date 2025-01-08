package frc.robot.Subsystems.AutoAlign;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public final class AutoAlignConstants {
    public static final Distance ROBOT_RADIUS = Meters.of(0.3);
    public static final Distance REEF_HITBOX = Meters.of(1.71); //1.66 m radius + 0.05 m for wiggle-room

    public static final double MIN_HEADING_ANGLE = -180;
    public static final double MAX_HEADING_ANGLE = 180;

    // TODO update max speed once robot is built
    public static final double MAX_SPEED = 4;
    public static final boolean USE_GOAL = true;
    public static final double DISTANCE_ERROR_MARGIN = .1;
    public static final double ANGLE_ERROR_MARGIN = .1;


    // TODO tune once we get the real robot
    public static final class Real{
        public static final PIDConstants TRANSLATIONAL_PID_CONSTANTS = new PIDConstants(1, 0, 0);
        public static final PIDConstants ROTATIONAL_PID_CONSTANTS = new PIDConstants(1, 0, 0);
    }

    public static final class Sim {
        public static final PIDConstants TRANSLATIONAL_PID_CONSTANTS = new PIDConstants(2.5, 0, 0);
        public static final PIDConstants ROTATIONAL_PID_CONSTANTS = new PIDConstants(0.8, 0, 0);
    }

    public static final class Poses {
        public static final class Red {
            public static final Pose2d REEF_POSE = new Pose2d(4.57, 4.09, new Rotation2d());
        }

        public static final class Blue {

        }

        public static final class Testing {
            public static final Pose2d test1 = new Pose2d(5.6, 1.67, new Rotation2d());
        }
    }


    public static final int TWO = 2; 
    public static final int ONE = 1;
    public static final int ZERO = 0;
}
