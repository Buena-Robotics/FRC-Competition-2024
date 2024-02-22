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
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AimAssistTarget;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.TunableNumber;

public class SwerveJoystickCmd extends Command {
    private static final TunableNumber left_joystick_deadband  = new TunableNumber("Drive/Joystick/LeftDeadband",0.2);
    private static final TunableNumber right_joystick_deadband = new TunableNumber("Drive/Joystick/RightDeadband",0.2);
    private static final double LEFT_DEADBAND_MODIFIER = 0.075;

    private final SwerveDrive swerve_drive;

    private final PIDController turret_turn_feedback;
    private final PIDController aim_assist_x_feedback, aim_assist_y_feedback;
    private final Supplier<Double> x_speed_function, y_speed_function, turn_speed_function;
    private final Supplier<Boolean> turret_mode;
    
    public SwerveJoystickCmd(SwerveDrive swerve_drive, Supplier<Double> x_speed_function, Supplier<Double> y_speed_function, Supplier<Double> turn_speed_function){
        this.swerve_drive = swerve_drive;
        
        this.turret_turn_feedback = new PIDController(5, 0, 0.1);
        this.turret_turn_feedback.enableContinuousInput(-Math.PI, Math.PI);
        this.aim_assist_x_feedback = new PIDController(0.75, 0, 0);
        this.aim_assist_y_feedback = new PIDController(0.75, 0, 0);
        this.x_speed_function    = x_speed_function;
        this.y_speed_function    = y_speed_function;
        this.turn_speed_function = turn_speed_function;
        
        this.turret_mode = Constants.IO.controller::getAButton;

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
        turn_speed *= Math.abs(Math.pow(turn_speed,2));

        x_speed *= SwerveDrive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        y_speed *= SwerveDrive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        turn_speed *= SwerveDrive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        
        ChassisSpeeds chassis_speeds = new ChassisSpeeds(x_speed, y_speed, turn_speed);

        final var best_aim_assist_target = FieldConstants.getBestAimAssistTarget(swerve_drive.getPose());
        if(best_aim_assist_target.getFirst() != null){
            final Pose2d robot_pose = swerve_drive.getPose();
            final Pose2d target_pose = best_aim_assist_target.getFirst().pose;
            final double distance_to_target = best_aim_assist_target.getSecond();
            if(distance_to_target < 1.5){
                final double rotation_offset = target_pose.getRotation().minus(robot_pose.getRotation()).getDegrees();
                if(Math.abs(rotation_offset) < 10){
                    final double x_distance = target_pose.getX() - robot_pose.getX();
                    final double y_distance = target_pose.getY() - robot_pose.getY();

                    double x_slow = Math.abs(aim_assist_x_feedback.calculate(0, x_distance));                                        
                    double y_slow = Math.abs(aim_assist_y_feedback.calculate(0, y_distance));
                    x_slow = MathUtil.clamp(x_slow, 0.05, 1);
                    y_slow = MathUtil.clamp(y_slow, 0.05, 1);

                    chassis_speeds = new ChassisSpeeds(x_speed * y_slow, y_speed * x_slow, turn_speed );
                }

            }
        }
        else if(turret_mode.get()){
            final double x1 = swerve_drive.getPose().getX(), 
                         y1 = swerve_drive.getPose().getY(), 
                         x2 = FieldConstants.getSpeakerPoint().x, 
                         y2 = FieldConstants.getSpeakerPoint().y;
            final double rise = y2-y1;
            final double run = x2-x1;
            final double distance = Math.sqrt(Math.pow(run, 2) + Math.pow(rise, 2));
            if(distance < Units.feetToMeters(20)){
                final double measurement = swerve_drive.getPose().getRotation().plus(getTurretRadianOffset(distance)).getRadians();
                final double setpoint = Math.atan(rise / run);
                turn_speed = turret_turn_feedback.calculate(setpoint, measurement);
                chassis_speeds = new ChassisSpeeds(x_speed, y_speed, turn_speed);
                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassis_speeds, swerve_drive.getPose().getRotation());
            }
        }
        SwerveModuleState[] module_states = swerve_drive.getKinematics().toSwerveModuleStates(chassis_speeds);
        swerve_drive.setModuleStates(module_states);
    }
    @Override public void end(boolean interrupted){ swerve_drive.stopModules(); }
    @Override public boolean isFinished(){ return false; }
}
