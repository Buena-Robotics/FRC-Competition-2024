package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIO;

public class PrepareLaunch extends Command {
    
    private ShooterIO shooter;

    public PrepareLaunch(ShooterIO shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override public void initialize() { shooter.launch_motor.set(ShooterIO.LAUNCH_SPEED); }

    @Override public void end(boolean interrupted) {}

    @Override public boolean isFinished() { return false; }
}
