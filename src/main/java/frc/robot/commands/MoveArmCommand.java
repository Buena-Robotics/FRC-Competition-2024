package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IO;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveArmCommand extends Command {
    
    private ClimbSubsystem climber;
    private double speed;
    private boolean shooting;

    public MoveArmCommand(ClimbSubsystem climber, Supplier<Double> speed, boolean shooting) {
        this.climber = climber;
        this.speed = speed.get();
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
         climber.moveArm(getClimberSpeed(), true, shooting);
         System.out.println(speed);
         super.execute();
     }

     private double getClimberSpeed() {
    if (IO.controller.getLeftTriggerAxis() > 0.01) 
      return -IO.controller.getLeftTriggerAxis();

    return IO.controller.getRightTriggerAxis();
  }
}
