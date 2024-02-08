package frc.robot.commands;

import java.util.Vector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.Drive;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToFieldPosCmd extends Command {
    private final static double X_METERS_AMBIGUITY = Units.inchesToMeters(4);
    private final static double Y_METERS_AMBIGUITY = Units.inchesToMeters(4);
    private final static double DEGREE_AMBIGUITY = 3;

    private final SwerveDriveSubsystem swerve_drive_subsystem;
    private final Pose2d target_pose;
    
    private Pose2d robot_pose;

    private double x_speed = 0;
    private double y_speed = 0;
    private double turn_speed = 0;

    private final PIDController x_pid_controller    = new PIDController(1, 0.5, 0);
    private final PIDController y_pid_controller    = new PIDController(1, 0.5, 0);
    private final PIDController turn_pid_controller = new PIDController(0.4, 0.02, 0);

    public DriveToFieldPosCmd(SwerveDriveSubsystem swerve_drive_subsystem, Pose2d target_pose) {
        this.swerve_drive_subsystem = swerve_drive_subsystem; 
        this.target_pose = target_pose;

        x_pid_controller.reset();
        x_pid_controller.setIZone(0.25);
        
        y_pid_controller.reset();
        y_pid_controller.setIZone(0.25);

        turn_pid_controller.reset();
        turn_pid_controller.enableContinuousInput(-Math.PI, Math.PI);
        x_pid_controller.setIZone(0.1);
    }
    
    public static Vector<Pose2d> getTrajectory(Pose2d target_pose){
        final PIDController x_pid_controller    = new PIDController(1, 0.5, 0);
        final PIDController y_pid_controller    = new PIDController(1, 0.5, 0);
        final PIDController turn_pid_controller = new PIDController(0.25, 0.02, 0);
        x_pid_controller.reset();
        x_pid_controller.setIZone(0.25);
        
        y_pid_controller.reset();
        y_pid_controller.setIZone(0.25);

        turn_pid_controller.reset();
        turn_pid_controller.enableContinuousInput(-Math.PI, Math.PI);
        x_pid_controller.setIZone(0.1);

        double x_speed = 0;
        double y_speed = 0;
        double turn_speed = 0;

        Vector<Pose2d> trajectory = new Vector<>();

        Pose2d start_robot_pose = SubSystems.swerve_drive_subsystem.getRobotPose();
        Pose2d robot_pose = SubSystems.swerve_drive_subsystem.getRobotPose();

        while(!(Math.abs(target_pose.getX() - robot_pose.getX()) < X_METERS_AMBIGUITY &&
                Math.abs(target_pose.getY() - robot_pose.getY()) < Y_METERS_AMBIGUITY && 
                Math.abs(target_pose.getRotation().getDegrees() - robot_pose.getRotation().getDegrees()) < DEGREE_AMBIGUITY)){
            robot_pose = SubSystems.swerve_drive_subsystem.getRobotPose();

            x_speed = x_pid_controller.calculate(robot_pose.getX(), target_pose.getX());
            y_speed = y_pid_controller.calculate(robot_pose.getY(), target_pose.getY());
            turn_speed = turn_pid_controller.calculate(robot_pose.getRotation().getRadians(), target_pose.getRotation().getRadians());
            
            ChassisSpeeds chassis_speeds;

            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, turn_speed, robot_pose.getRotation());
            Pose2d pose = SubSystems.swerve_drive_subsystem.getRobotPose().plus(new Transform2d(
                MathUtil.clamp(chassis_speeds.vxMetersPerSecond, -Units.feetToMeters(14.5), Units.feetToMeters(14.5)) / 50, 
                MathUtil.clamp(chassis_speeds.vyMetersPerSecond, -Units.feetToMeters(14.5), Units.feetToMeters(14.5)) / 50, 
                new Rotation2d(
                    MathUtil.clamp(
                        chassis_speeds.omegaRadiansPerSecond, 
                        -Drive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
                        Drive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND) / 50)
            ));
            SubSystems.swerve_drive_subsystem.setRobotPose(pose);
            trajectory.add(pose);
        }
        SubSystems.swerve_drive_subsystem.setRobotPose(start_robot_pose);
        x_pid_controller.close();
        y_pid_controller.close();
        turn_pid_controller.close();
        return trajectory;
    }

    @Override public void initialize() {}
    @Override public void execute() {
        robot_pose = swerve_drive_subsystem.getRobotPose();

        x_speed = x_pid_controller.calculate(robot_pose.getX(), target_pose.getX());
        y_speed = y_pid_controller.calculate(robot_pose.getY(), target_pose.getY());
        turn_speed = turn_pid_controller.calculate(robot_pose.getRotation().getRadians(), target_pose.getRotation().getRadians());
        
        ChassisSpeeds chassis_speeds;
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, turn_speed, robot_pose.getRotation());
        final SwerveModuleState[] moduleStates = SwerveDriveSubsystem.swerve_kinematics.toSwerveModuleStates(chassis_speeds);
        swerve_drive_subsystem.setModuleStates(moduleStates);


        SmartDashboard.putNumber("Xspeed", x_speed);
        SmartDashboard.putNumber("Yspeed", y_speed);
        SmartDashboard.putNumber("Turnspeed", turn_speed);

        SmartDashboard.putString("RobotPose", robot_pose.toString());
        SmartDashboard.putString("TargetPose", target_pose.toString());

        if(Robot.isSimulation()){
            swerve_drive_subsystem.setRobotPose(swerve_drive_subsystem.getRobotPose().plus(new Transform2d(
                MathUtil.clamp(chassis_speeds.vxMetersPerSecond, -Units.feetToMeters(14.5), Units.feetToMeters(14.5)) / 50, 
                MathUtil.clamp(chassis_speeds.vyMetersPerSecond, -Units.feetToMeters(14.5), Units.feetToMeters(14.5)) / 50, 
                new Rotation2d(MathUtil.clamp(chassis_speeds.omegaRadiansPerSecond, -Drive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, Drive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND) / 50))));
            swerve_drive_subsystem.getField2d().setRobotPose(swerve_drive_subsystem.getRobotPose());
        }

        swerve_drive_subsystem.getField2d().getObject("Tracking").setPoses(robot_pose, target_pose);
    }

    @Override public boolean isFinished() { 
        return  Math.abs(target_pose.getX() - robot_pose.getX()) < X_METERS_AMBIGUITY &&
                Math.abs(target_pose.getY() - robot_pose.getY()) < Y_METERS_AMBIGUITY && 
                Math.abs(target_pose.getRotation().getDegrees() - robot_pose.getRotation().getDegrees()) < DEGREE_AMBIGUITY;
    }

    @Override public void end(boolean interrupted) { }
}
