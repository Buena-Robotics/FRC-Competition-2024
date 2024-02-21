package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveArmCommand extends Command {
    
    private ClimbSubsystem climber;
    private Supplier<Double> speed;
    private boolean shooting;

    public MoveArmCommand(ClimbSubsystem climber, Supplier<Double> speed, boolean shooting) {
        this.climber = climber;
        this.speed = speed;
        this.shooting = shooting;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

     @Override
     public void execute() {
         climber.moveArm(speed.get(), true, shooting);
     }
}
