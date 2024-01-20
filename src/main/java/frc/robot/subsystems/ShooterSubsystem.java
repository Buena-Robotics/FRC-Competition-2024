package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    public CANSparkMax feedMotor, launchMotor;
    private final int feedId = 10, launchId = 9;
    private final int feedCurrentLimit = 80, launchCurrentLimit = 80;
    public final double launcherSpeed = 1, launchFeederSpeed = 1;
    public final double intakeLauncherSpeed = -1, intakeFeederSpeed = -.2;
    public final double launcherDelay = 0.6;

    public ShooterSubsystem() {
        feedMotor = new CANSparkMax(feedId, MotorType.kBrushless);
        launchMotor = new CANSparkMax(launchId, MotorType.kBrushless);

        feedMotor.setSmartCurrentLimit(feedCurrentLimit);
        launchMotor.setSmartCurrentLimit(launchCurrentLimit);
    }

    public Command intakeCommand() {
        return this.startEnd(() -> {
            feedMotor.set(intakeFeederSpeed);
            launchMotor.set(intakeLauncherSpeed);
        }, () -> {
            stop();
        });
    }

    public void stop() {
        feedMotor.set(0);
        launchMotor.set(0);
    }
}
