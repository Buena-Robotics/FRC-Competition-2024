package frc.robot.utils;

import java.util.Vector;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class FieldVisualizer {
    private static final Field2d field = new Field2d();    
    static {
        field.setRobotPose(new Pose2d());
        SmartDashboard.putData("Field", field);
    }
    public static void setAprilTags(AprilTagFieldLayout field_layout){
        Vector<Pose2d> tag_poses = new Vector<>(16);
        for(AprilTag tag : field_layout.getTags())
            tag_poses.add(tag.pose.toPose2d());
        field.getObject("AprilTags").setPoses(tag_poses);
    }
    public static void setFloorNotes(Vector<Pose2d> note_poses){ field.getObject("Notes").setPoses(note_poses); }
    public static void setTrajectory(Trajectory trajectory){ field.getObject("Trajectory").setTrajectory(trajectory); }
    public static Field2d getField(){ return field; }
}
