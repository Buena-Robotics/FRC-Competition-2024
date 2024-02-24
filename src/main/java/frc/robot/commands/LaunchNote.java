package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class LaunchNote extends Command {
    
    private Shooter shooter;

    public LaunchNote(Shooter shooter) { this.shooter = shooter; }

    @Override public void initialize() {
        shooter.launch_motor.set(Shooter.LAUNCH_SPEED);
        shooter.feed_motor.set(Shooter.FEED_SPEED);
    }
    @Override public boolean isFinished() { return false; }

    @Override public void end(boolean interrupted) { shooter.stop(); }
}
