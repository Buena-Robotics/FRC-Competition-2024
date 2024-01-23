package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.TimerUtil;

public class SwerveJoystickCmd extends Command {
    private static final double JOYSTICK_DEADBAND = 0.15;

    private final SwerveDriveSubsystem swerve_drive_subsystem;

    private final Supplier<Double>  x_speed_function, y_speed_function, turn_speed_function;
    private final Supplier<Boolean> field_orientation_function;

    private final SlewRateLimiter x_limiter, y_limiter, turn_limiter;

    private final TimerUtil turn_timer;

    private boolean should_zero_yaw;

    public SwerveJoystickCmd(SwerveDriveSubsystem swerve_drive_subsystem, Supplier<Double> x_speed_function, Supplier<Double> y_speed_function, Supplier<Double> turn_speed_function, Supplier<Boolean> field_orientation_function){
        this.swerve_drive_subsystem = swerve_drive_subsystem;
        
        this.x_speed_function    = x_speed_function;
        this.y_speed_function    = y_speed_function;
        this.turn_speed_function = turn_speed_function;
        
        this.field_orientation_function = field_orientation_function;
        
        this.x_limiter    = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.y_limiter    = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turn_limiter = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);

        this.turn_timer = new TimerUtil();
        addRequirements(swerve_drive_subsystem);
    }

    @Override public void initialize(){}
    @Override public void execute(){
        // 1. Get real-time joystick inputs
        double x_speed = x_speed_function.get();
        double y_speed = y_speed_function.get();
        double turn_speed = turn_speed_function.get();

        // 2. Apply deadband
        x_speed    = Math.abs(x_speed) > JOYSTICK_DEADBAND    ? x_speed : 0.0;
        y_speed    = Math.abs(y_speed) > JOYSTICK_DEADBAND    ? y_speed : 0.0;
        turn_speed = Math.abs(turn_speed) > 0.08 ? turn_speed : 0.0;

        // if(Math.abs(turn_speed) <= 0.01)
        //     turn_timer.reset();
        // if(turn_timer.hasTimeElapsed(250, true))
        //     should_zero_yaw = true;
        
        // if(should_zero_yaw && !swerve_drive_subsystem.gyro.isRotating()){
        //     swerve_drive_subsystem.zeroHeading();
        //     should_zero_yaw = false;
        // }
        
        // swerve_drive_subsystem.zeroHeading();
        // 3. Make the driving smoother
        x_speed    = x_limiter.calculate(x_speed) * Constants.Drive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        y_speed    = y_limiter.calculate(y_speed) * Constants.Drive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        turn_speed = turn_limiter.calculate(turn_speed) * Constants.Drive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassis_speeds;
        // if (field_orientation_function.get()){
            // Relative to field
            // chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, turn_speed, swerve_drive_subsystem.getRotation2d());
        // } else {
            // Relative to robot
            chassis_speeds = new ChassisSpeeds(x_speed, y_speed, turn_speed);
        // }

        SwerveModuleState[] moduleStates = Constants.Drive.DRIVE_KINEMATICS.toSwerveModuleStates(chassis_speeds);

        swerve_drive_subsystem.setModuleStates(moduleStates);
    }
    @Override public void end(boolean interrupted){ swerve_drive_subsystem.stopModules(); }
    @Override public boolean isFinished(){ return false; }
}
