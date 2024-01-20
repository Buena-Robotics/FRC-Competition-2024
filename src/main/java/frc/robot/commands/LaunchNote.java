package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchNote extends Command {
    
    private ShooterSubsystem shooter;

    public LaunchNote(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.launchMotor.set(shooter.launcherSpeed);
        shooter.feedMotor.set(shooter.launchFeederSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
