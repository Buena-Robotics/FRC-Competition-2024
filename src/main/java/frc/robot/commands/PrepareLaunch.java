package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareLaunch extends Command {
    
    private ShooterSubsystem shooter;

    public PrepareLaunch(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override public void initialize() { shooter.launch_motor.set(ShooterSubsystem.LAUNCH_SPEED); }

    @Override public void end(boolean interrupted) {}

    @Override public boolean isFinished() { return false; }
}
