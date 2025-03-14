package frc.robot.Subsystems.Vision;

import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

	private static VisionSystemSim visionSim;

	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
		super(name, robotToCamera);
		this.poseSupplier = poseSupplier;

		if (visionSim == null) {
			visionSim = new VisionSystemSim("main");
			visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
		}

		// Add sim camera
		// TODO: Uh like idrc about this ngl, but we can have a sim thats 4 deg more accurate if we want
		var cameraProperties = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(camera, cameraProperties);
		visionSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSim.update(poseSupplier.get());
		super.updateInputs(inputs);
	}
}
