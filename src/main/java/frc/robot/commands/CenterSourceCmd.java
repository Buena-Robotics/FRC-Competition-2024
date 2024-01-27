package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CenterSourceCmd extends Command {
    private final SwerveDriveSubsystem swerve_drive_subsystem;
    
    private Pose2d robot_pose;
    private Pose2d target_pose;

    private double x_speed = 0;
    private double y_speed = 0;
    private double turn_speed = 0;

    private final PIDController x_pid_controller    = new PIDController(0.15, 0, 0);
    private final PIDController y_pid_controller    = new PIDController(0.15, 0, 0);
    private final PIDController turn_pid_controller = new PIDController(0.75, 0, 0);
    private boolean test_end;

    public CenterSourceCmd(SwerveDriveSubsystem swerve_drive_subsystem) {
        this.swerve_drive_subsystem = swerve_drive_subsystem; 
        test_end = false;
        
        x_pid_controller.reset();
        
        y_pid_controller.reset();

        turn_pid_controller.reset();
        turn_pid_controller.enableContinuousInput(-Math.PI, Math.PI);
    }
    @Override public void initialize() {}
    @Override public void execute() {
        robot_pose = swerve_drive_subsystem.getRobotPose();
        target_pose = swerve_drive_subsystem.getTargetPose();

        if(Math.abs(robot_pose.getX() - target_pose.getX()) < 0.2)
            x_speed = 0;
        else
            x_speed = x_pid_controller.calculate(robot_pose.getX(), target_pose.getX());
        y_speed = y_pid_controller.calculate(robot_pose.getY(), target_pose.getY());
        turn_speed = turn_pid_controller.calculate(robot_pose.getRotation().getRadians(), -target_pose.getRotation().getRadians());

        x_speed = MathUtil.clamp(x_speed, -0.08, 0.08);
        y_speed = MathUtil.clamp(y_speed, -0.08, 0.08);
        turn_speed = MathUtil.clamp(turn_speed, -0.25, 0.25);

        y_speed *= -1;

        SmartDashboard.putNumber("Xspeed", x_speed);
        SmartDashboard.putNumber("Yspeed", y_speed);
        SmartDashboard.putNumber("Turnspeed", turn_speed);

        SmartDashboard.putString("robot_rot", robot_pose.getRotation().toString());
        SmartDashboard.putString("target_rot", target_pose.getRotation().toString());

        // if(Math.abs(turn_speed) > 0.03){
        //     final ChassisSpeeds chassis_speeds = new ChassisSpeeds(0, 0, turn_speed);
        //     final SwerveModuleState[] moduleStates = SwerveDriveSubsystem.swerve_kinematics.toSwerveModuleStates(chassis_speeds);
        //     swerve_drive_subsystem.setModuleStates(moduleStates);
        // }
        // else{
            final ChassisSpeeds chassis_speeds = new ChassisSpeeds(-x_speed, -y_speed, turn_speed);
            final SwerveModuleState[] moduleStates = SwerveDriveSubsystem.swerve_kinematics.toSwerveModuleStates(chassis_speeds);

            swerve_drive_subsystem.setModuleStates(moduleStates);
        // }

        // final Transform2d translation = target_pose.minus(robot_pose);
        // final double turn_speed = turn_pid_controller.calculate(robot_pose.getRotation().getRadians(), target_pose.getRotation().getRadians())

        // final ChassisSpeeds chassis_speeds = new ChassisSpeeds(-x_speed, -y_speed, -turn_speed);
        // final ChassisSpeeds chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, 0, swerve_drive_subsystem.getRotation2d());
        // final SwerveModuleState[] moduleStates = SwerveDriveSubsystem.swerve_kinematics.toSwerveModuleStates(chassis_speeds);

        // swerve_drive_subsystem.setModuleStates(moduleStates);
    }

    @Override public boolean isFinished() { return !swerve_drive_subsystem.getPoseKnown() || 
        Math.abs(robot_pose.getY() - target_pose.getY()) < 0.8; }

    @Override public void end(boolean interrupted) { }
}
