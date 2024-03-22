package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.SubSystems;

public class VisionCameraSim extends VisionCameraReal {
    private final VisionSystemSim vision_sim;
    private final PhotonCameraSim photon_camera_sim;
    
    public VisionCameraSim(String photon_camera_name, Transform3d robot_to_camera){
        super(photon_camera_name, robot_to_camera, PhotonPipeline.APRILTAG);

        vision_sim = new VisionSystemSim(photon_camera_name + "_sim");

        final SimCameraProperties camera_properties = new SimCameraProperties();

        camera_properties.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
        camera_properties.setCalibError(0.25, 0.08);
        camera_properties.setFPS(30);
        camera_properties.setAvgLatencyMs(20);
        camera_properties.setLatencyStdDevMs(5);

        photon_camera_sim = new PhotonCameraSim(photon_camera, camera_properties);

        vision_sim.addAprilTags(field_layout);
        vision_sim.addCamera(photon_camera_sim, robot_to_camera);
    }

    @Override public void periodic() {
        super.periodic();
        vision_sim.update(SubSystems.swerve_drive.getPose());
    }
}
