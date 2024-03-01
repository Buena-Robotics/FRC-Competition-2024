package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class PrepareLaunch extends Command {
    
    private Shooter shooter;

    public PrepareLaunch(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override public void initialize() { shooter.setLaunchVoltage(Shooter.LAUNCH_SPEED_MAX * 12); }

    @Override public void end(boolean interrupted) {}

    @Override public boolean isFinished() { return false; }
}
