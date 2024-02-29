package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldVisualizer;

public abstract class Vision extends SubsystemBase {
    protected final VisionCamera[] cameras;
    // public final Transform3d camera_pose = new Transform3d(
    //             Units.inchesToMeters(14.5),
    //             Units.inchesToMeters(14.5),
    //             Units.inchesToMeters(69/4.0),
    //             new Rotation3d(0,Units.degreesToRadians(-20), 0));

    public Vision(VisionCamera... cameras){
        FieldVisualizer.setAprilTags(VisionCamera.field_layout);
        this.cameras = cameras.clone();
    }

    @Override public void periodic() {
        super.periodic();
    }

    public void _periodic() {
        // ArrayList<Double> found_fidicual_ids = new ArrayList<Double>();

        // for(VisionCamera camera : cameras){
        //     var optional_vision_measurements = getVisionMeasurement(camera);
        //     if(optional_vision_measurements.isPresent()){
        //         vision_measurements.add(optional_vision_measurements.get());
        //     }
        // }
        // Double[] found_fidicual_ids_arr = new Double[found_fidicual_ids.size()];
        // found_fidicual_ids.toArray(found_fidicual_ids_arr);
        // SmartDashboard.putNumberArray("Vision/FoundIds", found_fidicual_ids_arr);
    }

    // public final ArrayList<VisionCamera> getCameras(){ return cameras; }
    // public void addCameras(VisionCamera... cameras){ for(var camera : cameras) this.cameras.add(camera); }
    // public List<Transform2d> getAllRobotToCameraTransforms(){
    //     List<Transform2d> transforms = new ArrayList<Transform2d>();
    //     for(VisionCamera camera : cameras)
    //         transforms.add(camera.getTransform2d());
    //     return transforms;
    // }
    // public List<Pose2d> getAllRobotToCameraPoses(Pose2d robot_pose){
    //     List<Transform2d> transforms = getAllRobotToCameraTransforms();
    //     List<Pose2d> poses = new ArrayList<Pose2d>(transforms.size());
    //     for(Transform2d transform : transforms)
    //         poses.add(robot_pose.transformBy(transform));
    //     return poses;
    // }
}