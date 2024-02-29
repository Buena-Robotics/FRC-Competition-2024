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

public class VisionCameraReal extends VisionCamera {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public VisionCameraReal(String photon_camera_name, Transform3d robot_to_camera){
        super(photon_camera_name, robot_to_camera);
    }

    private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        Matrix<N3, N1> estStdDevs = kSingleTagStdDevs;
        List<PhotonTrackedTarget> targets = photon_camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (PhotonTrackedTarget target : targets) {
            var tagPose = photon_pose_estimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    @Override public Optional<TimestampedVisionMeasurement> getVisionMeasurement(){
        if(pipeline != PhotonPipeline.APRILTAG || pipeline != PhotonPipeline.ARUCRO) return Optional.empty();
        if(!photon_camera.isConnected()) return Optional.empty();
        if(isCameraPipelineDisabled()) return Optional.empty();
        
        final Optional<EstimatedRobotPose> optional_estimated_pose = photon_pose_estimator.update();
        if(optional_estimated_pose.isEmpty())
            return Optional.empty();
        else{
            EstimatedRobotPose estimated_pose = optional_estimated_pose.get();
            if(previous_vision_result_timestamp == estimated_pose.timestampSeconds) return Optional.empty();
            previous_vision_result_timestamp = estimated_pose.timestampSeconds;
            return Optional.of(new TimestampedVisionMeasurement(
                estimated_pose.estimatedPose.toPose2d(), 
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
