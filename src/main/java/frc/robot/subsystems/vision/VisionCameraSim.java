package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionCameraSim extends VisionCameraReal {
    public VisionCameraSim(String photon_camera_name, Transform3d robot_to_camera){
        super(photon_camera_name, robot_to_camera);

        final PhotonCameraSim photon_camera_sim = new PhotonCameraSim(photon_camera); 
        final SimCameraProperties camera_properties = new SimCameraProperties();

        camera_properties.setCalibration(1280, 720, Rotation2d.fromDegrees(70)); //TODO: change calib FOV
        camera_properties.setCalibError(0.25, 0.08);
        camera_properties.setFPS(30);
        camera_properties.setAvgLatencyMs(20);
        camera_properties.setLatencyStdDevMs(5);
    }
}
