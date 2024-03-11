package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.SubSystems;

public abstract class VisionCamera {
    public static final AprilTagFieldLayout field_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    @AutoLog public static class VisionCameraInputs {
        public PhotonPipeline pipeline = PhotonPipeline.APRILTAG;
        public boolean driver_mode = false;
        
        public double result_timestamp_seconds = 0.0;
        public double result_latency_ms = 0.0;
        public double result_best_area = 0.0;
        public double result_best_ambiguity = 0.0;
        public double result_best_yaw = 0.0;
        public double result_best_pitch = 0.0;
        public double result_best_skew = 0.0;
        public int result_best_fidicual_id = -1;
    }
    public static class TimestampedVisionMeasurement {
        public final Pose3d pose;
        public final double timestamp;
        public final Matrix<N3, N1> std_devs;
        public final List<PhotonTrackedTarget> targets;
        public TimestampedVisionMeasurement(Pose3d pose, double timestamp, Matrix<N3, N1> std_devs, List<PhotonTrackedTarget> targets){ 
            this.pose = pose; this.timestamp = timestamp; this.std_devs = std_devs; this.targets = targets;
        }
    }
    public static enum PhotonPipeline {
        ARUCRO(0),
        APRILTAG(1),
        NOTE_DETECTION(2);
        
        private final int index;
        private PhotonPipeline(int index){ this.index = index; }
        public int getIndex(){ return index; }
        public static PhotonPipeline fromIndex(int index){
            switch(index) {
            case 0: return ARUCRO;
            case 1: return APRILTAG;
            case 2: return NOTE_DETECTION;
            default: return null;
            }
        }
    }

    protected final PhotonCamera photon_camera;
    protected final PhotonPoseEstimator photon_pose_estimator;
    protected VisionCameraInputsAutoLogged inputs = new VisionCameraInputsAutoLogged();
    protected PhotonPipeline pipeline = PhotonPipeline.ARUCRO;
    protected double previous_vision_result_timestamp = 0.0;
    public VisionCamera(String photon_camera_name, Transform3d robot_to_camera){
        this.photon_camera = new PhotonCamera(photon_camera_name);
        this.photon_pose_estimator = new PhotonPoseEstimator(field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photon_camera, robot_to_camera);
        this.photon_pose_estimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        this.photon_pose_estimator.setTagModel(TargetModel.kAprilTag36h11);
    }
    public abstract Optional<TimestampedVisionMeasurement> getVisionMeasurement();
    public abstract Optional<PhotonPipelineResult> getNoteDetection();

    public void updateInputs(){
        inputs.pipeline = PhotonPipeline.APRILTAG;
        inputs.driver_mode = false;
        
        final PhotonPipelineResult result = photon_camera.getLatestResult();
        
        inputs.result_timestamp_seconds = result.getTimestampSeconds();
        inputs.result_latency_ms = result.getLatencyMillis();

        if(result.hasTargets()){
            final PhotonTrackedTarget result_best = result.getBestTarget();
            inputs.result_best_area = result_best.getArea();
            inputs.result_best_ambiguity = result_best.getPoseAmbiguity();
            inputs.result_best_yaw = result_best.getYaw();
            inputs.result_best_pitch = result_best.getPitch();
            inputs.result_best_skew = result_best.getSkew();
            inputs.result_best_fidicual_id = result_best.getFiducialId();
        }
    }

    public void periodic(){
        updateInputs();
        Logger.processInputs("Vision/VisionCamera-" + photon_camera.getName(), inputs);
        Logger.recordOutput("Vision/VisionCamera-" + photon_camera.getName() + "/CameraPose", getCameraPoseOnRobot(SubSystems.swerve_drive.getPose()));
    }

    public void setPipeline(int index){ pipeline = PhotonPipeline.fromIndex(index); photon_camera.setPipelineIndex(index); }
    public void setEnablePipeline(boolean enable){ photon_camera.setDriverMode(!enable); }
    public void setCameraRotation(Rotation3d new_rotation){
        Transform3d current_transform = photon_pose_estimator.getRobotToCameraTransform();
        photon_pose_estimator.setRobotToCameraTransform(new Transform3d(current_transform.getTranslation(), new_rotation));
    }

    public boolean isCameraPipelineDisabled(){ return photon_camera.getDriverMode(); }
    public PhotonPipeline getPhotonPipeline(){ return pipeline; }
    public Pose3d getCameraPoseOnRobot(Pose2d robot_pose){
        Transform3d robot_to_camera = this.photon_pose_estimator.getRobotToCameraTransform();
        return new Pose3d(robot_pose).plus(robot_to_camera);
    }
}
