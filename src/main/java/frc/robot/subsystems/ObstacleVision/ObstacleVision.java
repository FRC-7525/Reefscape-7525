package frc.robot.Subsystems.ObstacleVision;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import java.util.ArrayList;

import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Subsystems.ObstacleVision.ObstacleVisionConstants.ObstaclePoseObservation;

public class ObstacleVision extends Subsystem<ObstacleVisionStates> {
    private static ObstacleVision instance;
    private ObstacleVisionIO[] io;

    private ArrayList<ObstaclePoseObservation> filteredObstacle;


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
        for (int i = 0; i < filteredObstacle.size(); i++) {
            if (Math.abs(Utils.fpgaToCurrentTime(filteredObstacle.get(i).timestamp) - Utils.getCurrentTimeSeconds()) > EXPIRATION_TIME)
                
            
        }


        for (ObstacleVisionIO camera: io) {
            for (ObstaclePoseObservation observation: camera.getObstaclePoses()) {
                boolean alreadyObserved = false;
                for (int i = 0; i < filteredObstacle.size(); i++) {
                    if (filteredObstacle.get(i).observedPose.getTranslation().getDistance(observation.observedPose.getTranslation()) < 0.4) {
                        alreadyObserved = true;
                        filteredObstacle.set(i, observation);
                        break;
                    }
                }
                if (alreadyObserved) filteredObstacle.add(observation);
            }
        }


    }
} 
