package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveArmCommand extends Command {
    
    private ClimbSubsystem climber;
    private double speed;
    private boolean shooting;

    public MoveArmCommand(ClimbSubsystem climber, double speed, boolean shooting) {
        this.climber = climber;
        this.speed = speed;
        this.shooting = shooting;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

     @Override
     public void execute() {
         climber.moveArm(speed, true, shooting);
         super.execute();
     }
}
