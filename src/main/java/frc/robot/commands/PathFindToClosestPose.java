package frc.robot.commands;

import java.util.HashSet;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.SwerveDrive;

public class PathFindToClosestPose {
    private Supplier<Pose2d> robot_pose_function;

    private double distanceToPose(Pose2d a, Pose2d b){
        return a.getTranslation().getDistance(b.getTranslation());
    }

    private Pose2d getClosestPose(final Pose2d robot_pose){
        Pose2d closest_pose = null;
        if(RobotState.armHasNote()) {
            closest_pose = FieldConstants.getAmpPathfindPose();
        }
        else if(!RobotState.shooterHasNote()){
            closest_pose = FieldConstants.getSourceCenterPathfindPose();
            if(distanceToPose(robot_pose, closest_pose) > distanceToPose(robot_pose, FieldConstants.getSourceLeftPathfindPose()))
                closest_pose = FieldConstants.getSourceLeftPathfindPose();
            if(distanceToPose(robot_pose, closest_pose) > distanceToPose(robot_pose, FieldConstants.getSourceRightPathfindPose()))
                closest_pose = FieldConstants.getSourceRightPathfindPose();
        }
        else { // Shooter has note  
            closest_pose = FieldConstants.getSpeakerCenterPathfindPose();
            if(distanceToPose(robot_pose, closest_pose) > distanceToPose(robot_pose, FieldConstants.getSpeakerLowerPathfindPose()))
                closest_pose = FieldConstants.getSpeakerLowerPathfindPose();
            if(distanceToPose(robot_pose, closest_pose) > distanceToPose(robot_pose, FieldConstants.getSpeakerUpperPathfindPose()))
                closest_pose = FieldConstants.getSpeakerUpperPathfindPose();
        }
        return closest_pose;
    }

    private Command pathFindToClosestPoseCommand(){
        PathConstraints constraints = new PathConstraints(
                3.8, 3.8,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(
            getClosestPose(robot_pose_function.get()),
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        return pathfindingCommand;
    }

    public Command pathFindToClosestPose(final SwerveDrive swerve_drive, Supplier<Pose2d> robot_pose_function){
        this.robot_pose_function = robot_pose_function;
        HashSet<Subsystem> subsystems = new HashSet<>(1);
        subsystems.add(swerve_drive);
        return Commands.defer(this::pathFindToClosestPoseCommand, subsystems);
    }
}
