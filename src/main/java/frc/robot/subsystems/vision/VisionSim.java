package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.FieldVisualizer;

public class VisionSim extends Vision {
    public final VisionSystemSim vision_sim;

    public VisionSim(){
        super();
        vision_sim = new VisionSystemSim("VisionSim");
        vision_sim.addAprilTags();

        VisionCamera vision_camera = this.cameras.get(0);

        PhotonCameraSim cameraSim = new PhotonCameraSim(vision_camera.photon_camera, camera_properties);
        
        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        vision_sim.addCamera(cameraSim, vision_camera.photon_pose_estimator.getRobotToCameraTransform());

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        cameraSim.enableDrawWireframe(true);
    }
    @Override public void periodic(){
        super.periodic();
        vision_sim.update(FieldVisualizer.getField().getRobotPose());
    }
}