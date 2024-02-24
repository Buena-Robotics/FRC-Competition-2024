package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldVisualizer;

public abstract class Vision extends SubsystemBase {
    public static final AprilTagFieldLayout field_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static class TimestampedVisionMeasurement {
        public final Pose2d pose;
        public final double timestamp;
        public TimestampedVisionMeasurement(Pose2d pose, double timestamp){ this.pose = pose; this.timestamp = timestamp; }
    }
    public static class VisionCamera {
        public final PhotonCamera photon_camera;
        public final PhotonPoseEstimator photon_pose_estimator;
        public double previous_vision_result_timestamp = 0.0;
        public ArrayList<Pose2d> tracked_target_poses = new ArrayList<Pose2d>();
        public VisionCamera(String photon_camera_name, Transform3d robot_to_camera){
            this.photon_camera = new PhotonCamera(photon_camera_name); 
            this.photon_pose_estimator = new PhotonPoseEstimator(field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robot_to_camera);
            this.photon_pose_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            this.photon_pose_estimator.setTagModel(TargetModel.kAprilTag36h11);
        }
        public Transform2d getTransform2d(){
            Transform3d robot_to_camera_3d = this.photon_pose_estimator.getRobotToCameraTransform();
            Translation2d translation2d = robot_to_camera_3d.getTranslation().toTranslation2d();
            Rotation2d rotation2d = robot_to_camera_3d.getRotation().toRotation2d();
            Transform2d robot_to_camera_2d = new Transform2d(translation2d, rotation2d);
            return robot_to_camera_2d;
        }
    }

    protected final ArrayList<VisionCamera> cameras = new ArrayList<VisionCamera>();
    protected final ArrayList<TimestampedVisionMeasurement> vision_measurements = new ArrayList<TimestampedVisionMeasurement>();

    public Vision(){
        FieldVisualizer.setAprilTags(field_layout);
        cameras.add(new VisionCamera("Microsoft_LifeCam_HD-3000", 
            new Transform3d(
                Units.inchesToMeters(14.5),
                Units.inchesToMeters(14.5),
                Units.inchesToMeters(69/4.0),
                new Rotation3d(0,Units.degreesToRadians(-30),0))));
    }

    protected abstract Optional<TimestampedVisionMeasurement> getVisionMeasurement(VisionCamera camera);

    public void _periodic() {
        for(VisionCamera camera : cameras){
            if(camera.photon_camera.isConnected()){
                var optional_vision_measurements = getVisionMeasurement(camera);
                if(optional_vision_measurements.isPresent())
                    vision_measurements.add(optional_vision_measurements.get());
            }
        }
    }

    public boolean hasVision(){
        for(var camera : cameras) if(camera.photon_camera.isConnected()) return true;
        return false;
    }
    public ArrayList<TimestampedVisionMeasurement> getVisionMeasurements(){ 
        var clone = new ArrayList<TimestampedVisionMeasurement>();
        clone.addAll(vision_measurements);
        vision_measurements.clear();
        return clone;
    };
    public final ArrayList<VisionCamera> getCameras(){ return cameras; }
    public void addCameras(VisionCamera... cameras){ for(var camera : cameras) this.cameras.add(camera); }
    public List<Transform2d> getAllRobotToCameraTransforms(){
        List<Transform2d> transforms = new ArrayList<Transform2d>();
        for(VisionCamera camera : cameras)
            transforms.add(camera.getTransform2d());
        return transforms;
    }
    public List<Pose2d> getAllRobotToCameraPoses(Pose2d robot_pose){
        List<Transform2d> transforms = getAllRobotToCameraTransforms();
        List<Pose2d> poses = new ArrayList<Pose2d>(transforms.size());
        for(Transform2d transform : transforms)
            poses.add(robot_pose.transformBy(transform));
        return poses;
    }
}