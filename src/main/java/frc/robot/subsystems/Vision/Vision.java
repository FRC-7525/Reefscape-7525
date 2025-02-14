package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Subsystems.Vision.VisionConstants.*;
import static frc.robot.subsystems.Vision.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.subsystems.Vision.VisionConstants.BACK_CAMERA_NAME;
import static frc.robot.subsystems.Vision.VisionConstants.BACK_RESOLUTION;
import static frc.robot.subsystems.Vision.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.subsystems.Vision.VisionConstants.FRONT_RESOLUTION;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_BACK_CAMERA;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_FRONT_CAMERA;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import com.google.flatbuffers.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.Vision.VisionIO.CameraPoseEstimator;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team7525.misc.VisionUtil.CameraResolution;

public class Vision extends SubsystemBase {

	VisionIO io;
	SwerveDrivePoseEstimator drivePoseEstimator;
	private final VisionIOInputsAutoLogged VisionInputs = new VisionIOInputsAutoLogged();

<<<<<<< HEAD
    /**
     * Constructor for the AprilTagVision subsystem. This is intended to be
     * instantiated in Drive.
     *
     * @param swerveDrivePoseEstimator A {@link SwerveDrivePoseEstimator} to write
     *                                 vision data to.
     *                                 Probably from Drive.
     */
    public Vision(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
        CameraPoseEstimator[] visionPoseEstimators = {};
  
                visionPoseEstimators = new CameraPoseEstimator[] {
                        new CameraPoseEstimator(
                                new PhotonCamera(BACK_CAMERA_NAME),
                                ROBOT_TO_BACK_CAMERA,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                BACK_RESOLUTION),
                        new CameraPoseEstimator(
                                new PhotonCamera(FRONT_CAMERA_NAME),
                                ROBOT_TO_FRONT_CAMERA,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                FRONT_RESOLUTION) //need to get resolution
                };
        
    

        switch (Constants.currentMode) {
            case REAL:
                this.io = new VisionIOReal(visionPoseEstimators);
                break;
            case REPLAY:
                this.io = new VisionIO() {
                };
                break;
            case SIM:
                this.io = new VisionIOSim();
                break;
        }
=======
	@AutoLogOutput
	public boolean useVision = true;

	/**
	 * Constructor for the AprilTagVision subsystem. This is intended to be
	 * instantiated in Drive.
	 *
	 * @param swerveDrivePoseEstimator A {@link SwerveDrivePoseEstimator} to write
	 *                                 vision data to.
	 *                                 Probably from Drive.
	 */
	public Vision(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
		CameraPoseEstimator[] visionPoseEstimators = {};
		switch (currentRobot) {
			case NAUTILUS:
				visionPoseEstimators = new CameraPoseEstimator[] {
					new CameraPoseEstimator(new PhotonCamera(BACK_CAMERA_NAME), ROBOT_TO_BACK_CAMERA, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BACK_RESOLUTION),
					new CameraPoseEstimator(new PhotonCamera(FRONT_CAMERA_NAME), ROBOT_TO_FRONT_CAMERA, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FRONT_RESOLUTION),
				};
			case DORY:
				visionPoseEstimators = new CameraPoseEstimator[] {
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.frontLeftName), DoryCameras.frontLeftFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.frontRightName), DoryCameras.frontRightFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.backLeftName), DoryCameras.backLeftFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
					new CameraPoseEstimator(new PhotonCamera(DoryCameras.backRightName), DoryCameras.backRightFromRobot, AprilTagConstants.poseStrategy, CameraResolution.NORMAL),
				};
		}
>>>>>>> 9b39c8408d93870afbdf9a1a0ba025ee20a39210

		switch (Constants.currentMode) {
			case REAL:
				this.io = new AprilTagVisionIOReal(visionPoseEstimators);
				break;
			case REPLAY:
				this.io = new AprilTagVisionIO() {};
				break;
			case SIM:
				this.io = new AprilTagVisionIOSim();
				break;
		}

		// For sim. If we do a full drivetrain sim move this there so it'll update as
		// the simulated
		// robot moves.
		io.updatePose(new Pose2d(1.06, 2.50, new Rotation2d(Radians.convertFrom(-114.0, Radians))));

<<<<<<< HEAD
    @Override
    public void periodic() {
        io.updateInputs(VisionInputs);
        Logger.processInputs("AprilTagVision", VisionInputs);

        for (int i = 0; i < VisionInputs.timestamps.length; i++) {
            if ( // Bounds check the estimated robot pose is actually on the field
            VisionInputs.timestamps[i] >= 1.0
                    && Math.abs(VisionInputs.visionPoses[i].getZ()) < 1.0
                    && VisionInputs.visionPoses[i].getX() > 0
                    && VisionInputs.visionPoses[i].getX() < APRIL_TAG_FIELD_LAYOUT.getFieldLength()
                    && VisionInputs.visionPoses[i].getY() > 0
                    && VisionInputs.visionPoses[i].getY() < APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
                    && VisionInputs.visionPoses[i].getRotation().getX() < 0.2
                    && VisionInputs.visionPoses[i].getRotation().getY() < 0.2) {
                if (VisionInputs.timestamps[i] > (getTimestamp() / 1.0e6)) {
                    VisionInputs.timestamps[i] = (getTimestamp() / 1.0e6) - VisionInputs.latency[i];
                }

                Logger.recordOutput(
                        "Drive/AprilTagPose" + i, VisionInputs.visionPoses[i].toPose2d());
                Logger.recordOutput(
                        "Drive/AprilTagStdDevs" + i,
                        Arrays.copyOfRange(VisionInputs.visionStdDevs, 3 * i, 3 * i + 3));
                Logger.recordOutput("Drive/AprilTagTimestamps" + i, VisionInputs.timestamps[i]);

                if (useVision) {
                    drivePoseEstimator.addVisionMeasurement(
                            VisionInputs.visionPoses[i].toPose2d(),
                            VisionInputs.timestamps[i],
                            VecBuilder.fill(
                                    VisionInputs.visionStdDevs[3 * i],
                                    VisionInputs.visionStdDevs[3 * i + 1],
                                    VisionInputs.visionStdDevs[3 * i + 2]));
                }
            } else {
                Logger.recordOutput("Drive/AprilTagPose" + i, new Pose2d());
                Logger.recordOutput("Drive/AprilTagStdDevs" + i, new double[] { 0.0, 0.0, 0.0 });
            }
        }
    }
}
=======
		this.drivePoseEstimator = swerveDrivePoseEstimator;
	}

	@Override
	public void periodic() {
		io.updateInputs(aprilTagVisionInputs);
		Logger.processInputs("AprilTagVision", aprilTagVisionInputs);

		for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
			if ( // Bounds check the estimated robot pose is actually on the field
				aprilTagVisionInputs.timestamps[i] >= 1.0 &&
				Math.abs(aprilTagVisionInputs.visionPoses[i].getZ()) < 1.0 &&
				aprilTagVisionInputs.visionPoses[i].getX() > 0 &&
				aprilTagVisionInputs.visionPoses[i].getX() < aprilTagFieldLayout.getFieldLength() &&
				aprilTagVisionInputs.visionPoses[i].getY() > 0 &&
				aprilTagVisionInputs.visionPoses[i].getY() < aprilTagFieldLayout.getFieldWidth() &&
				aprilTagVisionInputs.visionPoses[i].getRotation().getX() < 0.2 &&
				aprilTagVisionInputs.visionPoses[i].getRotation().getY() < 0.2
			) {
				if (aprilTagVisionInputs.timestamps[i] > (getTimestamp() / 1.0e6)) {
					aprilTagVisionInputs.timestamps[i] = (getTimestamp() / 1.0e6) - aprilTagVisionInputs.latency[i];
				}

				Logger.recordOutput("Drive/AprilTagPose" + i, aprilTagVisionInputs.visionPoses[i].toPose2d());
				Logger.recordOutput("Drive/AprilTagStdDevs" + i, Arrays.copyOfRange(aprilTagVisionInputs.visionStdDevs, 3 * i, 3 * i + 3));
				Logger.recordOutput("Drive/AprilTagTimestamps" + i, aprilTagVisionInputs.timestamps[i]);

				if (useVision) {
					drivePoseEstimator.addVisionMeasurement(aprilTagVisionInputs.visionPoses[i].toPose2d(), aprilTagVisionInputs.timestamps[i], VecBuilder.fill(aprilTagVisionInputs.visionStdDevs[3 * i], aprilTagVisionInputs.visionStdDevs[3 * i + 1], aprilTagVisionInputs.visionStdDevs[3 * i + 2]));
				}
			} else {
				Logger.recordOutput("Drive/AprilTagPose" + i, new Pose2d());
				Logger.recordOutput("Drive/AprilTagStdDevs" + i, new double[] { 0.0, 0.0, 0.0 });
			}
		}
	}
}
>>>>>>> 9b39c8408d93870afbdf9a1a0ba025ee20a39210
