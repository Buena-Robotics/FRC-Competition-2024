package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSim {
    public final VisionSystemSim vision_sim;


    public VisionSim(){
        vision_sim = new VisionSystemSim("VisionSim");
        // A 0.5 x 0.25 meter rectangular target
        TargetModel targetModel = new TargetModel(0.5, 0.25);
        // The pose of where the target is on the field.
        // Its rotation determines where "forward" or the target x-axis points.
        // Let's say this target is flat against the far wall center, facing the blue driver stations.
        Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        // The given target model at the given pose
        VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

        // Add this vision target to the vision system simulation to make it visible
        vision_sim.addVisionTargets(visionTarget);

        // The layout of AprilTags which we want to add to the vision system
        vision_sim.addAprilTags(Vision.field_layout);

        // The simulated camera properties
        SimCameraProperties cameraProp = new SimCameraProperties();

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        // The PhotonCamera used in the real robot code.
        PhotonCamera camera = new PhotonCamera("cameraName");

        // The simulation of this camera. Its values used in real robot code will be updated.
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        
        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        vision_sim.addCamera(cameraSim, robotToCamera);

        // The turret the camera is mounted on is rotated 5 degrees
        Rotation3d turretRotation = new Rotation3d(0, 0, Math.toRadians(5));
        robotToCamera = new Transform3d(
                robotToCameraTrl.rotateBy(turretRotation),
                robotToCameraRot.rotateBy(turretRotation));
        vision_sim.adjustCamera(cameraSim, robotToCamera);

        // Update with the simulated drivetrain pose. This should be called every loop in simulation.
        // vision_sim.update(robotPoseMeters);

        // Get the built-in Field2d used by this VisionSystemSim
        vision_sim.getDebugField();

        // Enable the raw and processed streams. These are enabled by default.
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        cameraSim.enableDrawWireframe(true);
    }
}