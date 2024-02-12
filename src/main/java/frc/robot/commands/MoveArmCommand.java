package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveArmCommand extends Command {
    
    private ClimbSubsystem climber;
    private double speed;

    public MoveArmCommand(ClimbSubsystem climber, double speed) {
        this.climber = climber;
        this.speed = speed;
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
         climber.moveArm(speed, true);
         super.execute();
     }
}
