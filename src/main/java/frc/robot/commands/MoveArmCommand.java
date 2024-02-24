package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climb;

public class MoveArmCommand extends Command {
    
    private Climb climber;
    private Supplier<Double> speed;
    private boolean shooting;

    public MoveArmCommand(Climb climber, Supplier<Double> speed, boolean shooting) {
        this.climber = climber;
        this.speed = speed;
        this.shooting = shooting;
        addRequirements(climber);
    }

    @Override public void execute() {
        climber.moveArm(speed.get(), true, shooting);
    }
}
