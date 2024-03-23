package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.drive.SwerveDrive;

public class LockOnNote extends Command {
    private final PIDController turn_feedback = new PIDController(0.5, 0, 0);
    private final Supplier<Optional<Pose2d>> pose_supplier = this::getClosestPose;
    private final SwerveDrive swerve_drive;
    private Optional<Rotation2d> initial_opt_setpoint;
    private Optional<Rotation2d> opt_setpoint;

    public LockOnNote(SwerveDrive swerve_drive){
        this.swerve_drive = swerve_drive;
        this.turn_feedback.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve_drive);
    }

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
    private Optional<Rotation2d> getRotationSetpoint(){
        Optional<Pose2d> opt_pose = pose_supplier.get();
        if(opt_pose.isEmpty()) return Optional.empty();
        
        final double y = opt_pose.get().getY() - swerve_drive.getPose().getY();
        final double x = opt_pose.get().getX() - swerve_drive.getPose().getX();
        final Rotation2d setpoint = new Rotation2d(Math.atan2(y, x));
        return Optional.of(setpoint);
    }

    @Override public void initialize() {
        initial_opt_setpoint = getRotationSetpoint();
    }
    @Override public void execute() {
        opt_setpoint = getRotationSetpoint();
        if(opt_setpoint.isPresent()){
            final Rotation2d measurement = swerve_drive.getPose().getRotation();
            final Rotation2d setpoint = opt_setpoint.get();
            double omega_speed = turn_feedback.calculate(measurement.getRadians(), setpoint.getRadians());
            omega_speed /= Math.PI; // 
            omega_speed *= 3;
            omega_speed *= SwerveDrive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
            
            final ChassisSpeeds chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, omega_speed, swerve_drive.getRotation2d().plus(swerve_drive.getRotationOffset()));
            
            swerve_drive.driveRobotVelocity(chassis_speeds);
        }
        
    }
    @Override public void end(boolean interrupted) {

    }
    @Override public boolean isFinished() {
        return opt_setpoint.isEmpty()
            || Math.abs(swerve_drive.getPose().getRotation().minus(getRotationSetpoint().get()).getDegrees()) <= 3;
    }
}
