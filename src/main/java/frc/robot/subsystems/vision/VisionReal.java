package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionReal extends Vision {
    private TimestampedVisionMeasurement getVisionMeasurementMultiTarget(VisionCamera camera, PhotonPipelineResult pipeline_result, double result_timestamp){
        final MultiTargetPNPResult multitag_result = pipeline_result.getMultiTagResult();
        final List<Integer> fiducial_ids = multitag_result.fiducialIDsUsed;
        final Transform3d field_to_camera = multitag_result.estimatedPose.best;
        final Pose3d camera_pose = new Pose3d(field_to_camera.getTranslation(), field_to_camera.getRotation());
        final Pose3d vision_measurement = camera_pose.transformBy(camera.photon_pose_estimator.getRobotToCameraTransform());
        final TimestampedVisionMeasurement timestamped_vision_measurement = new TimestampedVisionMeasurement(vision_measurement.toPose2d(), result_timestamp);

        for(int fiducial_id : fiducial_ids)
            camera.tracked_target_poses.add(field_layout.getTagPose(fiducial_id).get().toPose2d());
        return timestamped_vision_measurement;
    }
    private Optional<TimestampedVisionMeasurement> getVisionMeasurementSingleTarget(VisionCamera camera, PhotonPipelineResult pipeline_result, double result_timestamp){
        final PhotonTrackedTarget target = pipeline_result.getBestTarget();
        final int fiducial_id = target.getFiducialId();
        final boolean tag_pose_exists = field_layout.getTagPose(fiducial_id).isPresent();
        if(target.getPoseAmbiguity() <= 0.2 && tag_pose_exists){
            final Pose3d target_pose = field_layout.getTagPose(fiducial_id).get();
            final Transform3d camera_to_target = target.getBestCameraToTarget();
            final Pose3d camera_pose = target_pose.transformBy(camera_to_target.inverse());

            final Pose3d vision_measurement = camera_pose.transformBy(camera.photon_pose_estimator.getRobotToCameraTransform());
            final TimestampedVisionMeasurement timestamped_vision_measurement = new TimestampedVisionMeasurement(vision_measurement.toPose2d(), result_timestamp);

            camera.tracked_target_poses.add(field_layout.getTagPose(fiducial_id).get().toPose2d());
            return Optional.of(timestamped_vision_measurement);
        }
        return Optional.empty();
    }
    
    @Override protected Optional<TimestampedVisionMeasurement> getVisionMeasurement(VisionCamera camera){
        camera.tracked_target_poses.clear();

        if(!camera.photon_camera.isConnected()) return Optional.empty();

        final PhotonPipelineResult pipeline_result = camera.photon_camera.getLatestResult();
        final double result_timestamp = pipeline_result.getTimestampSeconds();

        if(result_timestamp != camera.previous_vision_result_timestamp && pipeline_result.hasTargets()){
            camera.previous_vision_result_timestamp = result_timestamp;
            if(pipeline_result.getMultiTagResult().estimatedPose.isPresent) // Multi Tag Detection
                return Optional.of(getVisionMeasurementMultiTarget(camera, pipeline_result, result_timestamp));
            else // Single Tag Detection
                return getVisionMeasurementSingleTarget(camera, pipeline_result, result_timestamp);
        }
        return Optional.empty();
    }
}
