package frc.robot.subsystems.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.SwerveDriveIO;

import java.util.List;

import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class SwerveDrive extends SwerveDriveIO {
    public SwerveDrive(){
        super();
    }

    @Override public Pose2d getTargetPose(){ return trajectory_target_pose; }

    @Override public void setVisionMeasurementMultiTarget(PhotonPipelineResult pipeline_result, double result_timestamp){
        final MultiTargetPNPResult multitag_result = pipeline_result.getMultiTagResult();
        final List<Integer> fiducial_ids = multitag_result.fiducialIDsUsed;
        final Transform3d field_to_camera = multitag_result.estimatedPose.best;
        final Pose3d camera_pose = new Pose3d(field_to_camera.getTranslation(), field_to_camera.getRotation());

        final Pose3d vision_measurement = camera_pose.transformBy(CAMERA_TO_ROBOT);
        swerve_pose_estimator.addVisionMeasurement(vision_measurement.toPose2d(), result_timestamp);

        for(int fiducial_id : fiducial_ids)
            tracked_target_poses.add(field_layout.getTagPose(fiducial_id).get().toPose2d());
    }
    @Override public void setVisionMeasurementSingleTarget(PhotonPipelineResult pipeline_result, double result_timestamp){
        final PhotonTrackedTarget target = pipeline_result.getBestTarget();
        final int fiducial_id = target.getFiducialId();
        final boolean tag_pose_exists = field_layout.getTagPose(fiducial_id).isPresent();
        if(target.getPoseAmbiguity() <= 0.2 && tag_pose_exists){
            final Pose3d target_pose = field_layout.getTagPose(fiducial_id).get();
            final Transform3d camera_to_target = target.getBestCameraToTarget();
            final Pose3d camera_pose = target_pose.transformBy(camera_to_target.inverse());

            final Pose3d vision_measurement = camera_pose.transformBy(CAMERA_TO_ROBOT);
            swerve_pose_estimator.addVisionMeasurement(vision_measurement.toPose2d(), result_timestamp);
        }
        if(tag_pose_exists)
            tracked_target_poses.add(field_layout.getTagPose(fiducial_id).get().toPose2d());
    }
    
    @Override public void setVisionMeasurement(){
        tracked_target_poses.clear();

        if(!photon_camera.isConnected()) return;

        final PhotonPipelineResult pipeline_result = photon_camera.getLatestResult();
        final double result_timestamp = pipeline_result.getTimestampSeconds();

        if(result_timestamp != previous_vision_result_timestamp && pipeline_result.hasTargets()){
            previous_vision_result_timestamp = result_timestamp;
            if(pipeline_result.getMultiTagResult().estimatedPose.isPresent) // Multi Tag Detection
                setVisionMeasurementMultiTarget(pipeline_result, result_timestamp);
            else // Single Tag Detection
                setVisionMeasurementSingleTarget(pipeline_result, result_timestamp);
        }
    }

    @Override public void updatePoseEstimator(){
        setVisionMeasurement();

        swerve_pose_estimator.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                front_right.getPosition(),
                front_left.getPosition(),
                back_right.getPosition(),
                back_left.getPosition()
        });
    }

    @Override public void periodic() { 
        updatePoseEstimator();
        glass_field.setRobotPose(getRobotPose());
    }
}
