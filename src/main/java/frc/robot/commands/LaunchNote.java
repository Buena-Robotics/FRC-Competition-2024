package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIO;

public class LaunchNote extends Command {
    
    private ShooterIO shooter;

    public LaunchNote(ShooterIO shooter) { this.shooter = shooter; }

    @Override public void initialize() {
        shooter.launch_motor.set(ShooterIO.LAUNCH_SPEED);
        shooter.feed_motor.set(ShooterIO.FEED_SPEED);
    }
    @Override public boolean isFinished() { return false; }

    @Override public void end(boolean interrupted) { shooter.stop(); }
}
