package frc.robot.commands;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.drive.SwerveDrive;

public class PathFindNote extends Command {
    private double distanceFrom(Pose2d a, Pose2d b){
        return a.getTranslation().getDistance(b.getTranslation());
    }

    private Optional<Pose2d> getClosestPose(){
        final List<Pose2d> note_poses = SubSystems.vision.getAllDetectedNoteEstimatedPoses2d();
        final Pose2d robot_pose = SubSystems.swerve_drive.getPose();

        Optional<Pose2d> closest_note_pose = Optional.empty();

        for(Pose2d note_pose: note_poses){
            if(closest_note_pose.isPresent()){
                if(distanceFrom(robot_pose, note_pose) < distanceFrom(robot_pose, closest_note_pose.get()))
                    closest_note_pose = Optional.of(note_pose);
            }
            else closest_note_pose = Optional.of(note_pose);
        }
        return closest_note_pose;
    }
    private Command pathFindToNote(){
        PathConstraints constraints = new PathConstraints(
                1.0, 0.5,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        Optional<Pose2d> closest_pose_opt = getClosestPose();
        Command command = new WaitCommand(0);
        if(closest_pose_opt.isPresent()){
            command = AutoBuilder.pathfindToPose(
                closest_pose_opt.get(),
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            );
        }
        return command;
    }

    public Command pathFindToClosestNote(final SwerveDrive swerve_drive){
        HashSet<Subsystem> subsystems = new HashSet<>(1);
        subsystems.add(swerve_drive);
        return Commands.defer(this::pathFindToNote, subsystems);
    }
}
