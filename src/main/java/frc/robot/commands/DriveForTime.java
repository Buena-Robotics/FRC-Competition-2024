package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.TimerUtil;

public class DriveForTime extends Command {
    public static final Rotation2d DIR_TOP = Rotation2d.fromDegrees(120);
    public static final Rotation2d DIR_CENTER = Rotation2d.fromDegrees(180);
    public static final Rotation2d DIR_BOTTOM = Rotation2d.fromDegrees(-120);

    private final TimerUtil timer = new TimerUtil();
    private final SwerveDrive swerve_drive;
    private final Rotation2d direction;
    private final long time_ms;

    public DriveForTime(SwerveDrive swerve_drive, Rotation2d direction, long time_ms){
        this.swerve_drive = swerve_drive;
        this.direction = direction;
        this.time_ms = time_ms;
        addRequirements(swerve_drive);
    }

    @Override public void initialize() { timer.reset(); }
    @Override public void execute() {
        final Translation2d linear_velocity =
            new Pose2d(Constants.Empty.TRANSL2D_ZERO, direction)
                .transformBy(new Transform2d(Units.feetToMeters(6), 0.0, Constants.Empty.R2D_ZERO))
                .getTranslation();
        
        final ChassisSpeeds chassis_speeds = new ChassisSpeeds(linear_velocity.getX(), linear_velocity.getY(), 0);
        swerve_drive.driveRobotVelocity(chassis_speeds);
    }
    @Override public void end(boolean interrupted) { swerve_drive.xStopModules(); }
    @Override public boolean isFinished() { return timer.hasTimeElapsed(time_ms, true); }
}
