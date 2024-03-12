package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterTurret extends Command {
    private final Climb climb;

    public ShooterTurret(final Climb climb){
        this.climb = climb;

        addRequirements(climb);
    }

    @Override public void execute() { climb.runSetpoint(Shooter.estimatedShooterRotation()); }
}
