package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.IO;
import frc.robot.FieldConstants.AimAssistTarget;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.TunableNumber;

public class SwerveJoystickCmd extends Command {
    private static final TunableNumber left_joystick_deadband  = new TunableNumber("Joystick/LeftDeadband",0.1);
    private static final TunableNumber right_joystick_deadband = new TunableNumber("Joystick/RightDeadband",0.1);
    private static final double LEFT_DEADBAND_MODIFIER = 0.075;

    private final SwerveDrive swerve_drive;

    private final PIDController turret_turn_feedback;
    private final PIDController aim_assist_x_feedback, aim_assist_y_feedback;
    private final Supplier<Double> x_speed_function, y_speed_function, turn_speed_function;
    private final Supplier<Boolean> turret_mode;
    private boolean field_oriented_mode = false;
    
    public SwerveJoystickCmd(SwerveDrive swerve_drive, Supplier<Double> x_speed_function, Supplier<Double> y_speed_function, Supplier<Double> turn_speed_function){
        this.swerve_drive = swerve_drive;
        
        this.turret_turn_feedback = new PIDController(5, 0, 0.1);
        this.turret_turn_feedback.enableContinuousInput(-Math.PI, Math.PI);
        this.aim_assist_x_feedback = new PIDController(0.75, 0, 0);
        this.aim_assist_y_feedback = new PIDController(0.75, 0, 0);
        this.x_speed_function    = x_speed_function;
        this.y_speed_function    = y_speed_function;
        this.turn_speed_function = turn_speed_function;
        
        this.turret_mode = Constants.IO.controller::getStartButton;
        IO.commandController.back().onTrue(new InstantCommand(
            () -> {
                    swerve_drive.navx.zeroYaw();
                    field_oriented_mode = !field_oriented_mode; }
            ));

        addRequirements(swerve_drive);
    }

    @Override public void initialize(){}

    public Rotation2d getTurretRadianOffset(double distance_meters){
        return Rotation2d.fromRadians(Math.sqrt(distance_meters)/(2 * Math.PI)); 
    }

    @Override public void execute(){
        double x_speed = x_speed_function.get();
        double y_speed = y_speed_function.get();
        double turn_speed = turn_speed_function.get();

        if(Math.abs(x_speed) > 0.9)
             y_speed = Math.abs(y_speed) > left_joystick_deadband.get() + LEFT_DEADBAND_MODIFIER ? y_speed : 0.0;
        else y_speed = Math.abs(y_speed) > left_joystick_deadband.get() ? y_speed : 0.0;

        if(Math.abs(y_speed) > 0.9)
             x_speed = Math.abs(x_speed) > left_joystick_deadband.get() + LEFT_DEADBAND_MODIFIER ? x_speed : 0.0;
        else x_speed = Math.abs(x_speed) > left_joystick_deadband.get() ? x_speed : 0.0;

        turn_speed = Math.abs(turn_speed) > right_joystick_deadband.get() ? turn_speed : 0.0;

        x_speed *= Math.abs(Math.pow(x_speed,3));
        y_speed *= Math.abs(Math.pow(y_speed,3));
        turn_speed *= Math.abs(Math.pow(turn_speed,5));

        x_speed *= SwerveDrive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        y_speed *= SwerveDrive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        turn_speed *= SwerveDrive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        
        ChassisSpeeds chassis_speeds = new ChassisSpeeds(x_speed, y_speed, turn_speed);

        if(field_oriented_mode){
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, turn_speed, swerve_drive.navx.getRotation2d());
        }

        // final var best_aim_assist_target = FieldConstants.getBestAimAssistTarget(swerve_drive.getPose());
        // if(best_aim_assist_target.getFirst() != null){
        //     final Pose2d robot_pose = swerve_drive.getPose();
        //     final Pose2d target_pose = best_aim_assist_target.getFirst().pose;
        //     final double distance_to_target = best_aim_assist_target.getSecond();
        //     if(distance_to_target < 1.5){
        //         final double rotation_offset = target_pose.getRotation().minus(robot_pose.getRotation()).getDegrees();
        //         if(Math.abs(rotation_offset) < 10){
        //             final double x_distance = target_pose.getX() - robot_pose.getX();
        //             final double y_distance = target_pose.getY() - robot_pose.getY();

        //             double x_slow = Math.abs(aim_assist_x_feedback.calculate(0, x_distance));                                        
        //             double y_slow = Math.abs(aim_assist_y_feedback.calculate(0, y_distance));
        //             x_slow = MathUtil.clamp(x_slow, 0.05, 1);
        //             y_slow = MathUtil.clamp(y_slow, 0.05, 1);

        //             chassis_speeds = new ChassisSpeeds(x_speed * y_slow, y_speed * x_slow, turn_speed );
        //         }

        //     }
        // }

        
        SwerveModuleState[] module_states = swerve_drive.getKinematics().toSwerveModuleStates(chassis_speeds);
        swerve_drive.setModuleStates(module_states);
    }
    @Override public void end(boolean interrupted){ swerve_drive.stopModules(); }
    @Override public boolean isFinished(){ return false; }
}
