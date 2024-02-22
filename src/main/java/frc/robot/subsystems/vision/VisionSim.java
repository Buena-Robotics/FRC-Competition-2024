package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.vision.Vision.VisionCamera;
import frc.robot.utils.FieldVisualizer;

public class VisionSim extends VisionReal {
    public final VisionSystemSim vision_sim;

    public VisionSim(){
        super();
        vision_sim = new VisionSystemSim("VisionSim");
        vision_sim.addAprilTags(Vision.field_layout);

        SimCameraProperties camera_properties = new SimCameraProperties();

        camera_properties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        camera_properties.setCalibError(0.25, 0.08);
        camera_properties.setFPS(30);
        camera_properties.setAvgLatencyMs(20);
        camera_properties.setLatencyStdDevMs(5);

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