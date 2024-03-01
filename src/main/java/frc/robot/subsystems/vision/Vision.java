package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionCamera.TimestampedVisionMeasurement;
import frc.robot.utils.FieldVisualizer;

public class Vision extends SubsystemBase {
    public static class CameraData {
        public final String camera_name;
        public final Transform3d camera_transform;
        public CameraData(String camera_name, Transform3d camera_transform){
            this.camera_name = camera_name; this.camera_transform = camera_transform;
        }
    }
    private final Map<String, VisionCamera> camera_map = new HashMap<String, VisionCamera>();
    // public final Transform3d camera_pose = new Transform3d(
    //             Units.inchesToMeters(14.5),
    //             Units.inchesToMeters(14.5),
    //             Units.inchesToMeters(69/4.0),
    //             new Rotation3d(0,Units.degreesToRadians(-20), 0));

    public Vision(CameraData... camera_datas){
        FieldVisualizer.setAprilTags(VisionCamera.field_layout);
        for(var camera_data : camera_datas){
            if(Robot.isReal())
                camera_map.put(camera_data.camera_name, new VisionCameraReal(camera_data.camera_name, camera_data.camera_transform));
            else 
                camera_map.put(camera_data.camera_name, new VisionCameraSim(camera_data.camera_name, camera_data.camera_transform));
        }
    }

    @Override public void periodic() {
        for(VisionCamera camera : getAllCameras()) camera.periodic();
    }

    public VisionCamera getCamera(String key){ return camera_map.get(key); }
    public List<VisionCamera> getAllCameras(){ return new ArrayList<>(camera_map.values()); }
    public List<TimestampedVisionMeasurement> getAllVisionMeasurements(){
        ArrayList<TimestampedVisionMeasurement> vision_measurements = new ArrayList<TimestampedVisionMeasurement>();
        for(VisionCamera camera : getAllCameras()){
            var optional_measurement = camera.getVisionMeasurement();
            if(optional_measurement.isPresent())
                vision_measurements.add(optional_measurement.get());
        }
        return vision_measurements;
    }
    public List<PhotonPipelineResult> getAllDetectedNotes(){
        ArrayList<PhotonPipelineResult> detected_notes = new ArrayList<PhotonPipelineResult>();
        for(VisionCamera camera : getAllCameras()){
            var optional_detection = camera.getNoteDetection();
            if(optional_detection.isPresent())
                detected_notes.add(optional_detection.get());
        }
        return detected_notes;
    }


        // ArrayList<Double> found_fidicual_ids = new ArrayList<Double>();

        // for(VisionCamera camera : cameras){
        //     var optional_vision_measurements = getVisionMeasurement(camera);
        //     if(optional_vision_measurements.isPresent()){
        //         vision_measurements.add(optional_vision_measurements.get());
        //     }
        // }
        // Double[] found_fidicual_ids_arr = new Double[found_fidicual_ids.size()];
        // found_fidicual_ids.toArray(found_fidicual_ids_arr);
        // SmartDashboard.putNumberArray("Vision/FoundIds", found_fidicual_ids_arr);

    // public final ArrayList<VisionCamera> getCameras(){ return cameras; }
    // public void addCameras(VisionCamera... cameras){ for(var camera : cameras) this.cameras.add(camera); }
    // public List<Transform2d> getAllRobotToCameraTransforms(){
    //     List<Transform2d> transforms = new ArrayList<Transform2d>();
    //     for(VisionCamera camera : cameras)
    //         transforms.add(camera.getTransform2d());
    //     return transforms;
    // }
    // public List<Pose2d> getAllRobotToCameraPoses(Pose2d robot_pose){
    //     List<Transform2d> transforms = getAllRobotToCameraTransforms();
    //     List<Pose2d> poses = new ArrayList<Pose2d>(transforms.size());
    //     for(Transform2d transform : transforms)
    //         poses.add(robot_pose.transformBy(transform));
    //     return poses;
    // }
}