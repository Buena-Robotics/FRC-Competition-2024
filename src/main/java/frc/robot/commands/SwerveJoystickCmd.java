package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDriveIO;

public class SwerveJoystickCmd extends Command {
    private static final double LEFT_JOYSTICK_DEADBAND = 0.1;
    private static final double RIGHT_JOYSTICK_DEADBAND = 0.1;

    private final SwerveDriveIO swerve_drive;

    private final Supplier<Double>  x_speed_function, y_speed_function, turn_speed_function;

    private final SlewRateLimiter x_limiter, y_limiter, turn_limiter;

    public SwerveJoystickCmd(SwerveDriveIO swerve_drive, Supplier<Double> x_speed_function, Supplier<Double> y_speed_function, Supplier<Double> turn_speed_function){
        this.swerve_drive = swerve_drive;
        
        this.x_speed_function    = x_speed_function;
        this.y_speed_function    = y_speed_function;
        this.turn_speed_function = turn_speed_function;
        
        // this.field_orientation_function = field_orientation_function;
        
        this.x_limiter    = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.y_limiter    = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turn_limiter = new SlewRateLimiter(Constants.Drive.TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        
        addRequirements(swerve_drive);
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

        SwerveModuleState[] moduleStates = swerve_drive.getKinematics().toSwerveModuleStates(chassis_speeds);
        
        swerve_drive.setModuleStates(moduleStates);

        if(Robot.isSimulation())
            swerve_drive.getGyro().setAngleAdjustment(swerve_drive.getHeading() - chassis_speeds.omegaRadiansPerSecond);;
    }
    @Override public void end(boolean interrupted){ swerve_drive.stopModules(); }
    @Override public boolean isFinished(){ return false; }
}
