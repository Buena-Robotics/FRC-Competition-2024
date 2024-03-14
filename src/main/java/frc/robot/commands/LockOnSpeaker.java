package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;

public class LockOnSpeaker extends Command {
    private final PIDController turn_feedback = new PIDController(0.5, 0, 0);
    private final SwerveDrive swerve_drive;
    private final Climb climb;

    public LockOnSpeaker(final SwerveDrive swerve_drive, final Climb climb){
        this.swerve_drive = swerve_drive;
        this.climb = climb;
        this.turn_feedback.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(climb);
    }

    private Rotation2d getDriveRotationSetpoint(){
        final double y = FieldConstants.getSpeakerPoint().getY() - swerve_drive.getPose().getY();
        final double x = FieldConstants.getSpeakerPoint().getX() - swerve_drive.getPose().getX();
        final Rotation2d setpoint = new Rotation2d(Math.atan2(y, x)).minus(Rotation2d.fromDegrees(15));
        return setpoint;
    }

    @Override public void execute() { 
        climb.runSetpoint(Shooter.estimatedShooterRotation());

        final Rotation2d measurement = swerve_drive.getPose().getRotation();
        final Rotation2d setpoint = getDriveRotationSetpoint();
        double omega_speed = turn_feedback.calculate(measurement.getRadians(), setpoint.getRadians());
        omega_speed /= Math.PI; // 
        omega_speed *= 3;
        omega_speed *= SwerveDrive.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
        
        final ChassisSpeeds chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, omega_speed, swerve_drive.getRotation2d().plus(swerve_drive.getRotationOffset()));
        
        swerve_drive.driveRobotVelocity(chassis_speeds);
    }
    @Override public boolean isFinished() {
        final Rotation2d measurement = swerve_drive.getPose().getRotation();
        final Rotation2d setpoint = getDriveRotationSetpoint();

        return Math.abs(climb.getShooterAngleRotation().minus(Shooter.estimatedShooterRotation()).getDegrees()) <= 2
            && Math.abs(setpoint.minus(measurement).getDegrees()) <= 2;
    }
}
