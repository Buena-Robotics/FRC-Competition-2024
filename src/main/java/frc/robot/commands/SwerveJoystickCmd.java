package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveJoystickCmd extends Command {
    private static final double LEFT_JOYSTICK_DEADBAND = 0.08;
    private static final double RIGHT_JOYSTICK_DEADBAND = 0.08;

    private final SwerveDriveSubsystem swerve_drive_subsystem;

    private final Supplier<Double>  x_speed_function, y_speed_function, turn_speed_function;
    // private final Supplier<Boolean> field_orientation_function;

    private final SlewRateLimiter x_limiter, y_limiter, turn_limiter;

    public SwerveJoystickCmd(SwerveDriveSubsystem swerve_drive_subsystem, Supplier<Double> x_speed_function, Supplier<Double> y_speed_function, Supplier<Double> turn_speed_function){
        this.swerve_drive_subsystem = swerve_drive_subsystem;
        
        this.x_speed_function    = x_speed_function;
        this.y_speed_function    = y_speed_function;
        this.turn_speed_function = turn_speed_function;
        
        // this.field_orientation_function = field_orientation_function;
        
        this.x_limiter    = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.y_limiter    = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turn_limiter = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        

        addRequirements(swerve_drive_subsystem);
    }

    @Override public void initialize(){}
    @Override public void execute(){
        // 1. Get real-time joystick inputs
        double x_speed = x_speed_function.get();
        double y_speed = y_speed_function.get();
        double turn_speed = turn_speed_function.get();

        // 2. Apply deadband
        x_speed    = Math.abs(x_speed)    > LEFT_JOYSTICK_DEADBAND    ? x_speed    : 0.0;
        y_speed    = Math.abs(y_speed)    > LEFT_JOYSTICK_DEADBAND    ? y_speed    : 0.0;
        turn_speed = Math.abs(turn_speed) > RIGHT_JOYSTICK_DEADBAND   ? turn_speed : 0.0;

        // 3. Make the driving smoother
        x_speed    = x_limiter.calculate(x_speed) * Constants.Drive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        y_speed    = y_limiter.calculate(y_speed) * Constants.Drive.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        turn_speed = turn_limiter.calculate(turn_speed) * Constants.Drive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        ChassisSpeeds chassis_speeds = new ChassisSpeeds(x_speed, y_speed, turn_speed);

        SwerveModuleState[] moduleStates = SwerveDriveSubsystem.swerve_kinematics.toSwerveModuleStates(chassis_speeds);
        
        swerve_drive_subsystem.setModuleStates(moduleStates);

        if(Robot.isSimulation()){
            swerve_drive_subsystem.setRobotPose(swerve_drive_subsystem.getRobotPose().plus(new Transform2d(chassis_speeds.vxMetersPerSecond / 50, chassis_speeds.vyMetersPerSecond / 50, new Rotation2d(chassis_speeds.omegaRadiansPerSecond / 50))));
            swerve_drive_subsystem.getField2d().setRobotPose(swerve_drive_subsystem.getRobotPose());
        }
    }
    @Override public void end(boolean interrupted){ swerve_drive_subsystem.stopModules(); }
    @Override public boolean isFinished(){ return false; }
}
