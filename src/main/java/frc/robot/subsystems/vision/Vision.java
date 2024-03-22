package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.vision.VisionCamera.PhotonPipeline;
import frc.robot.subsystems.vision.VisionCamera.TimestampedVisionMeasurement;
import frc.robot.utils.FieldVisualizer;
import frc.robot.utils.Print;

public class Vision extends SubsystemBase {
    public static class CameraData {
        public final String camera_name;
        public final Transform3d camera_transform;
        public final PhotonPipeline pipeline;
        public CameraData(String camera_name, Transform3d camera_transform, PhotonPipeline pipeline){
            this.camera_name = camera_name; this.camera_transform = camera_transform; this.pipeline = pipeline;
        }
    }
    private final Map<String, VisionCamera> camera_map = new HashMap<String, VisionCamera>();

    public Vision(CameraData... camera_datas){
        FieldVisualizer.setAprilTags(VisionCamera.field_layout);
        for(var camera_data : camera_datas){
            if(Robot.isReal())
                camera_map.put(camera_data.camera_name, new VisionCameraReal(camera_data.camera_name, camera_data.camera_transform, camera_data.pipeline));
            else 
                camera_map.put(camera_data.camera_name, new VisionCameraSim(camera_data.camera_name, camera_data.camera_transform));
        }
    }

    @Override public void periodic() {
        List<VisionCamera> all_cameras = getAllCameras();
        for(int i = 0; i < all_cameras.size(); i++) { 
            all_cameras.get(i).periodic(); 
            if(!all_cameras.get(i).isConnected()){
                Print.error("'Camera {%s}' Not Connected [Vision]", all_cameras.get(i).getName());
            }
        }
        getAllDetectedNoteEstimatedPoses();
    }

    public VisionCamera getCamera(String key){ return camera_map.get(key); }
    public List<VisionCamera> getAllCameras(){ return new ArrayList<VisionCamera>(camera_map.values()); }
    
    ArrayList<TimestampedVisionMeasurement> vision_measurements = new ArrayList<TimestampedVisionMeasurement>();
    public List<TimestampedVisionMeasurement> getAllVisionMeasurements(){
        vision_measurements.clear();
        List<VisionCamera> all_cameras = getAllCameras();
        for(int i = 0; i < all_cameras.size(); i++){
            var optional_measurement = all_cameras.get(i).getVisionMeasurement();
            if(optional_measurement.isPresent())
                vision_measurements.add(optional_measurement.get());
        }
        return vision_measurements;
    }

    ArrayList<PhotonPipelineResult> detected_notes = new ArrayList<PhotonPipelineResult>();
    public List<PhotonPipelineResult> getAllDetectedNotes(){
        detected_notes.clear();
        List<VisionCamera> all_cameras = getAllCameras();
        for(int i = 0; i < all_cameras.size(); i++){
            var optional_detection = all_cameras.get(i).getNoteDetection();
            if(optional_detection.isPresent())
                detected_notes.add(optional_detection.get());
        }
        return detected_notes;
    }

    final int MAX_TIMESTEPS = 100;
    final List<Pose3d> note_poses = new ArrayList<Pose3d>(5);
    final List<Pose2d> note_poses2d = new ArrayList<Pose2d>(5);
    public List<Pose3d> getAllDetectedNoteEstimatedPoses(){
        note_poses.clear();
        note_poses2d.clear();

        for(PhotonPipelineResult result : getAllDetectedNotes()){
            for(PhotonTrackedTarget target : result.targets){         
                   
                Pose3d current_pose = camera_map.get("USB_Camera").getCameraPoseOnRobot(SubSystems.swerve_drive.getPose());
                // TODO: Check this
                current_pose = current_pose.rotateBy(new Rotation3d(0, Units.degreesToRadians(target.getPitch()), 
                    Units.degreesToRadians(target.getYaw())));
                // current_pose = current_pose.transformBy(new Transform3d(0,0,0, new Rotation3d(0, Units.degreesToRadians(target.getPitch()), 
                   // Units.degreesToRadians(target.getYaw()))));
                for(int i = 0; current_pose.getZ() > 0 && i < MAX_TIMESTEPS; i++){
                    current_pose = current_pose.transformBy(new Transform3d(Units.inchesToMeters(1),0,0, Constants.Empty.R3D_ZERO));
                }
                note_poses.add(new Pose3d(
                        current_pose.toPose2d().getX(), 
                        current_pose.toPose2d().getY(),
                        Units.inchesToMeters(1), Constants.Empty.R3D_ZERO));
                note_poses2d.add(current_pose.toPose2d());
            }
        }
        Logger.recordOutput("Vision/Note Poses", note_poses.toArray(new Pose3d[note_poses.size()]) );
        Logger.recordOutput("Vision/Note Poses2d", note_poses2d.toArray(new Pose2d[note_poses2d.size()]) );

        return note_poses;
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