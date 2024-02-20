package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoCommand extends Command {
    
    public SwerveDriveSubsystem swerveDrive;
    public ClimbSubsystem climber;
    public ShooterSubsystem shooter;

    public AutoCommand(SwerveDriveSubsystem swerveDrive, ClimbSubsystem climber, ShooterSubsystem shooter) {
        this.swerveDrive = swerveDrive;
        this.climber = climber;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        super.execute();
    }
}
