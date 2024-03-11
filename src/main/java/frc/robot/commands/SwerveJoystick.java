package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.TunableNumber;

public class SwerveJoystick extends Command {
    private static final TunableNumber left_joystick_deadband  = new TunableNumber("Joystick/LeftDeadband",0.1);
    private static final TunableNumber right_joystick_deadband = new TunableNumber("Joystick/RightDeadband",0.1);

    private final SwerveDrive swerve_drive;

    private final PIDController turret_turn_feedback;
    private final Supplier<Double> x_speed_function, y_speed_function, turn_speed_function;
    private final Supplier<Boolean> turret_mode;
    private final Supplier<Boolean> field_oriented_mode;
    
    public SwerveJoystick(SwerveDrive swerve_drive, Supplier<Double> x_speed_function, Supplier<Double> y_speed_function, Supplier<Double> turn_speed_function, Supplier<Boolean> field_oriented_mode, Supplier<Boolean> turret_mode){
        this.swerve_drive = swerve_drive;
        
        this.turret_turn_feedback = new PIDController(1, 0, 0);
        this.turret_turn_feedback.enableContinuousInput(-Math.PI, Math.PI);
        this.x_speed_function    = x_speed_function;
        this.y_speed_function    = y_speed_function;
        this.turn_speed_function = turn_speed_function;
        this.field_oriented_mode = field_oriented_mode;
        
        this.turret_mode = turret_mode;

        addRequirements(swerve_drive);
    }

    @Override public void initialize(){}

    public Rotation2d getTurretRadianOffset(double distance_meters){
        return Rotation2d.fromRadians(Math.sqrt(distance_meters)/(2 * Math.PI)); 
    }

    @Override public void execute() {
        double linear_magnitude =
                    MathUtil.applyDeadband(
                        Math.hypot(x_speed_function.get(), y_speed_function.get()), left_joystick_deadband.get());
        Rotation2d linear_direction =
            new Rotation2d(x_speed_function.get(), y_speed_function.get());
        double omega = MathUtil.applyDeadband(turn_speed_function.get(), right_joystick_deadband.get());

        linear_magnitude *= linear_magnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linear_velocity =
            new Pose2d(new Translation2d(), linear_direction)
                .transformBy(new Transform2d(linear_magnitude, 0.0, new Rotation2d()))
                .getTranslation();

        double x_speed = linear_velocity.getX(),
                y_speed = linear_velocity.getY(), 
                omega_speed = omega;

        x_speed *= Math.abs(Math.pow(x_speed,1));
        y_speed *= Math.abs(Math.pow(y_speed,1));
        omega_speed *= Math.abs(Math.pow(omega_speed,2));

        x_speed *= SwerveDrive.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        y_speed *= SwerveDrive.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        omega_speed *= SwerveDrive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        ChassisSpeeds chassis_speeds = new ChassisSpeeds(x_speed, y_speed, omega_speed);

        if(turret_mode.get()){
            final Rotation2d measurement = swerve_drive.getPose().getRotation();
            final double y = FieldConstants.getSpeakerPoint().getY() - swerve_drive.getPose().getY();
            final double x = FieldConstants.getSpeakerPoint().getX() - swerve_drive.getPose().getX();
            final Rotation2d setpoint = new Rotation2d(Math.atan2(y, x)).minus(Rotation2d.fromDegrees(15));
            omega_speed = turret_turn_feedback.calculate(measurement.getRadians(), setpoint.getRadians());
            omega_speed /= Math.PI; // 
            omega_speed *= 3;
            omega_speed *= SwerveDrive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, omega_speed, swerve_drive.getRotation2d().plus(swerve_drive.getRotationOffset()));
        } else if(field_oriented_mode.get()){
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, omega_speed, swerve_drive.getRotation2d().plus(swerve_drive.getRotationOffset()));
        }

        swerve_drive.driveRobotVelocity(chassis_speeds);
    }

    @Override public void end(boolean interrupted){ swerve_drive.stopModules(); }
    @Override public boolean isFinished(){ return false; }
}
