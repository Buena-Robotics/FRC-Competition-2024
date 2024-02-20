package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    }

    private final ArrayList<VisionCamera> cameras = new ArrayList<VisionCamera>();
    private final ArrayList<TimestampedVisionMeasurement> vision_measurements = new ArrayList<TimestampedVisionMeasurement>();

    public Vision(){
        FieldVisualizer.setAprilTags(field_layout);
        cameras.add(new VisionCamera("Microsoft_LifeCam_HD-3000", new Transform3d()));
    }

    protected abstract Optional<TimestampedVisionMeasurement> getVisionMeasurement(VisionCamera camera);

    @Override public void periodic() {
        vision_measurements.clear();
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
    public void addCameras(VisionCamera... cameras){ for(var camera : cameras) this.cameras.add(camera); }
    public ArrayList<TimestampedVisionMeasurement> getVisionMeasurements(){ return vision_measurements; };
}