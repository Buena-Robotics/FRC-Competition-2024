package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.TimerUtil;

public class PrepareLaunch extends Command {
    
    private ShooterSubsystem shooter;
    private TimerUtil timer;

    public PrepareLaunch(ShooterSubsystem shooter) {
        this.shooter = shooter;
        timer = new TimerUtil();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.launchMotor.set(shooter.launcherSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("bruh");
        // while (!timer.hasTimeElapsed(1000, false)) {
        //     shooter.launchMotor.set(shooter.launcherSpeed);
        //     shooter.feedMotor.set(shooter.launchFeederSpeed);
        // }

        // shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
