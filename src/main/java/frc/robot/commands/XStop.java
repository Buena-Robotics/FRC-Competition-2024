package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.TimerUtil;

public class XStop extends Command {
    private TimerUtil timer = new TimerUtil();
    private final SwerveDrive swerve_drive;
    private final long time_ms;

    public XStop(final SwerveDrive swerve_drive, long time_ms){
        this.swerve_drive = swerve_drive;
        this.time_ms = time_ms;

        addRequirements(swerve_drive);
    }

    @Override public void initialize() { timer.reset(); }
    @Override public void execute() { swerve_drive.stopModules(); }
    @Override public boolean isFinished() { return timer.hasTimeElapsed(time_ms, true); }
}
