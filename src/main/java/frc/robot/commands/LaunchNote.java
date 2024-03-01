package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class LaunchNote extends Command {
    
    private Shooter shooter;

    public LaunchNote(Shooter shooter) { this.shooter = shooter; }

    @Override public void initialize() {
        shooter.setLaunchVoltage(LAUNCH_SPEED_MAX * 12);
        shooter.setFeedVoltage(LAUNCH_SPEED_MAX * 12);
    }
    @Override public boolean isFinished() { return false; }

    @Override public void end(boolean interrupted) { shooter.stop(); }
}
