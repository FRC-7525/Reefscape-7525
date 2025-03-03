package frc.robot.Subsystems.ObstacleVision;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.team7525.subsystem.Subsystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ObstacleVision extends Subsystem<ObstacleVisionStates> {
    private static ObstacleVision instance;
    
    private ObstacleVisionIO[] io;

    private ObstacleVision(ObstacleVisionIO... io) {
        super("ObstacleVision", ObstacleVisionStates.ON);
        this.io = io;
    }

	public static ObstacleVision getInstance() {
		if (instance == null) {
			instance = new ObstacleVision(
                switch (ROBOT_MODE) {
                    case REAL -> new ObstacleVisionIOReal[] {
                        new ObstacleVisionIOReal("test", new Transform3d())
                    };
                    case SIM -> new ObstacleVisionIOSim[] {
                        new ObstacleVisionIOSim("test", new Transform3d(0, 0, 0, new Rotation3d(1, 1, -Math.PI/2)))
                    };
                    case TESTING -> new ObstacleVisionIOReal[] {
                        new ObstacleVisionIOReal("test", new Transform3d())
                    };
                });

        }
		return instance;
	}

    @Override
    protected void runState() {
        // TODO Auto-generated method stub
        for (ObstacleVisionIO camera: io) {
            camera.getObstaclePoses();
        }
    }
} 
