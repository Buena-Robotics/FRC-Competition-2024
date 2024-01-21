package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    private static final int FEED_MOTOR_ID = 10, LAUNCH_MOTOR_ID = 9;
    private static final int FEED_SMART_CURRENT_LIMIT = 80, LAUNCH_SMART_CURRENT_LIMIT = 80;
    public static final double FEED_SPEED = 1, LAUNCH_SPEED = 1;
    public static final double INTAKE_LAUNCH_SPEED = -1, INTAKE_FEED_SPEED = -.2;

    public CANSparkMax feed_motor, launch_motor;
    public RelativeEncoder feed_motor_encoder, launch_motor_encoder;

    public ShooterSubsystem() {
        feed_motor = new CANSparkMax(FEED_MOTOR_ID, MotorType.kBrushless);
        launch_motor = new CANSparkMax(LAUNCH_MOTOR_ID, MotorType.kBrushless);

        feed_motor_encoder = feed_motor.getEncoder();
        launch_motor_encoder = launch_motor.getEncoder();

        feed_motor.setSmartCurrentLimit(FEED_SMART_CURRENT_LIMIT);
        launch_motor.setSmartCurrentLimit(LAUNCH_SMART_CURRENT_LIMIT);
    }

    public double getFeedMotorVelocity(){ return feed_motor_encoder.getVelocity(); }
    public double getLaunchMotorVelocity(){ return feed_motor_encoder.getVelocity(); }

    public Command intakeCommand() {
        return this.startEnd(() -> {
            feed_motor.set(INTAKE_FEED_SPEED);
            launch_motor.set(INTAKE_LAUNCH_SPEED);
        }, () -> { stop(); });
    }

    public void stop() {
        feed_motor.set(0);
        launch_motor.set(0);
    }

    @Override public void periodic(){}
}
