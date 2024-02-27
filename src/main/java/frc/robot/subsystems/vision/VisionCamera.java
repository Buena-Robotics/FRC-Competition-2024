package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public abstract class VisionCamera {
    public static final AprilTagFieldLayout field_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static class TimestampedVisionMeasurement {
        public final Pose2d pose;
        public final double timestamp;
        public final Matrix<N3, N1> std_devs;
        public final List<PhotonTrackedTarget> targets;
        public TimestampedVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> std_devs, List<PhotonTrackedTarget> targets){ 
            this.pose = pose; this.timestamp = timestamp; this.std_devs = std_devs; this.targets = targets;
        }
    }
    protected final PhotonCamera photon_camera;
    protected final PhotonPoseEstimator photon_pose_estimator;
    protected double previous_vision_result_timestamp = 0.0;
    public VisionCamera(String photon_camera_name, Transform3d robot_to_camera){
        this.photon_camera = new PhotonCamera(photon_camera_name);
        this.photon_camera.setDriverMode(true);
        this.photon_pose_estimator = new PhotonPoseEstimator(field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photon_camera, robot_to_camera);
        this.photon_pose_estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        this.photon_pose_estimator.setTagModel(TargetModel.kAprilTag36h11);
    }
    public abstract Optional<TimestampedVisionMeasurement> getVisionMeasurement();

    public void setEnablePipeline(boolean enable){ photon_camera.setDriverMode(!enable); }

    public Pose3d getCameraPoseOnRobot(Pose2d robot_pose){
        Transform3d robot_to_camera = this.photon_pose_estimator.getRobotToCameraTransform();
        return new Pose3d(robot_pose).plus(robot_to_camera);
    }
}
