package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climb;

public class ShooterTurret extends Command {
    private final Climb climb;
    private final Supplier<Rotation2d> setpoint;

    public ShooterTurret(final Climb climb, final Supplier<Rotation2d> setpoint){
        this.climb = climb;
        this.setpoint = setpoint;
    }

    @Override public void execute() { climb.runSetpoint(setpoint.get()); }
}
