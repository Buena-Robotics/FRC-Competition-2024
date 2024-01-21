package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchNote extends Command {
    
    private ShooterSubsystem shooter;

    public LaunchNote(ShooterSubsystem shooter) { this.shooter = shooter; }

    @Override public void initialize() {
        shooter.launch_motor.set(ShooterSubsystem.LAUNCH_SPEED);
        shooter.feed_motor.set(ShooterSubsystem.FEED_SPEED);
    }
    //TODO Check if this still works
    @Override public boolean isFinished() { return true; } // WAS RETURN FALSE

    @Override public void end(boolean interrupted) { shooter.stop(); }
}
