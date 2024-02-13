package frc.robot.subsystems.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveIO;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SwerveDriveSim extends SwerveDriveIO {
    VisionSystemSim visionSim = new VisionSystemSim("main");
    SimCameraProperties cameraProp = new SimCameraProperties();
    PhotonCameraSim cameraSim = new PhotonCameraSim(photon_camera, cameraProp);

    public SwerveDriveSim(){
        super();

        TargetModel targetModel = new TargetModel(0.5, 0.25);

        Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        // The given target model at the given pose
        VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
        visionSim.addVisionTargets(visionTarget);

        visionSim.addAprilTags(field_layout);

        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(80));
        cameraProp.setCalibError(0.1, 0.01);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(15);
        cameraProp.setLatencyStdDevMs(5);

        // visionSim.addCamera(cameraSim, CAMERA_TO_ROBOT);

        // cameraSim.enableProcessedStream(true);
        // cameraSim.enableDrawWireframe(true);
    }

    @Override public Pose2d getTargetPose(){ return trajectory_target_pose; }

    @Override public void setVisionMeasurementMultiTarget(PhotonPipelineResult pipeline_result, double result_timestamp){
        final MultiTargetPNPResult multitag_result = pipeline_result.getMultiTagResult();
        final Transform3d field_to_camera = multitag_result.estimatedPose.best;
        final Pose3d camera_pose = new Pose3d(field_to_camera.getTranslation(), field_to_camera.getRotation());

        final Pose3d vision_measurement = camera_pose.transformBy(CAMERA_TO_ROBOT);
        swerve_pose_estimator.addVisionMeasurement(vision_measurement.toPose2d(), result_timestamp);

        
        for(PhotonTrackedTarget target : pipeline_result.targets){
            final Optional<Pose3d> tag_pose = field_layout.getTagPose(target.getFiducialId());
            if(target.getPoseAmbiguity() <= 0.1 && target.getPoseAmbiguity() >= 0 && tag_pose.isPresent())
                tracked_target_poses.add(tag_pose.get().toPose2d());
        }
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
            SmartDashboard.putBoolean("Multi Tag?", pipeline_result.getMultiTagResult().estimatedPose.isPresent);
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
        visionSim.update(getRobotPose());
    }
}
