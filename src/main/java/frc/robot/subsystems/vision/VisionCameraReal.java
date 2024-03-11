package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionCameraReal extends VisionCamera {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 8, 16);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public VisionCameraReal(String photon_camera_name, Transform3d robot_to_camera){
        super(photon_camera_name, robot_to_camera);
    }

    private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimated_pose) {
        if(DriverStation.isDisabled()){
            return VecBuilder.fill(0.5, 0.5, 2); // Not moving
        }
        List<PhotonTrackedTarget> targets = photon_camera.getLatestResult().getTargets();
        int tag_count = 0;
        double average_distance = 0;
        double average_ambiguity = 0;
        for (PhotonTrackedTarget target : targets) {
            var tag_pose = photon_pose_estimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tag_pose.isEmpty()) continue;
            tag_count++;
            average_distance += tag_pose.get().toPose2d().getTranslation().getDistance(estimated_pose.getTranslation());
            average_ambiguity += target.getPoseAmbiguity() == -1 ? 1 : target.getPoseAmbiguity();
        }
        // Should be unreachable
        if (tag_count == 0) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        average_distance /= tag_count;
        average_ambiguity /= tag_count;

        final double ambiguity_factor = average_ambiguity > 0.2 ? average_ambiguity * 16 : 0.8;

        // Multi-tag
        if (tag_count > 1) 
            return VecBuilder.fill(0.5 * ambiguity_factor, 0.5 * ambiguity_factor * 1.25, 1 * ambiguity_factor * 1.5);
        // Single tag > 4 meters away
        if (tag_count == 1 && average_distance > 4)
            return VecBuilder.fill(4 * ambiguity_factor, 8 * ambiguity_factor, 16 * ambiguity_factor);
        return VecBuilder.fill(2 * ambiguity_factor, 3 * ambiguity_factor, 6 * ambiguity_factor);
    }

    @Override public Optional<TimestampedVisionMeasurement> getVisionMeasurement(){
        // if(pipeline != PhotonPipeline.APRILTAG || pipeline != PhotonPipeline.ARUCRO) return Optional.empty();
        if(!photon_camera.isConnected()) return Optional.empty();
        // if(isCameraPipelineDisabled()) return Optional.empty();
        
        final Optional<EstimatedRobotPose> optional_estimated_pose = photon_pose_estimator.update();
        if(optional_estimated_pose.isEmpty())
            return Optional.empty();
        else{
            EstimatedRobotPose estimated_pose = optional_estimated_pose.get();
            if(previous_vision_result_timestamp == estimated_pose.timestampSeconds) return Optional.empty();
            previous_vision_result_timestamp = estimated_pose.timestampSeconds;
            return Optional.of(new TimestampedVisionMeasurement(
                estimated_pose.estimatedPose, 
                estimated_pose.timestampSeconds,
                getEstimationStdDevs(estimated_pose.estimatedPose.toPose2d()),
                estimated_pose.targetsUsed ));
        }
    }
    @Override public Optional<PhotonPipelineResult> getNoteDetection() {
        if(pipeline != PhotonPipeline.NOTE_DETECTION) return Optional.empty();
        if(!photon_camera.isConnected()) return Optional.empty();
        if(isCameraPipelineDisabled()) return Optional.empty();

        if(!photon_camera.getLatestResult().hasTargets()) return Optional.empty();

        return Optional.of(photon_camera.getLatestResult());
    }
}
