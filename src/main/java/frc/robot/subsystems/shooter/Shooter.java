package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    @AutoLog public static class ShooterInputs {
        public double intake_velocity_radians_per_second = 0.0;
        public double intake_velocity_rotations_per_second = 0.0;
        public double intake_applied_volts = 0.0;
        public double[] intake_current_amps = new double[] {};
        public double[] intake_temp_celcius = new double[] {};

        public double outake_velocity_radians_per_second = 0.0;
        public double outake_velocity_rotations_per_second = 0.0;
        public double outake_applied_volts = 0.0;
        public double[] outake_current_amps = new double[] {};
        public double[] outake_temp_celcius = new double[] {};
    }

    protected static final int FEED_MOTOR_ID = 10;
    protected static final int LAUNCH_MOTOR_ID = 9;
    protected static final int FEED_SMART_CURRENT_LIMIT = 80; 
    protected static final int LAUNCH_SMART_CURRENT_LIMIT = 80;
    public static final double FEED_SPEED = 1;
    public static final double LAUNCH_SPEED = 1;
    public static final double INTAKE_LAUNCH_SPEED = -1; 
    public static final double INTAKE_FEED_SPEED = -.2;

    public final CANSparkMax feed_motor = new CANSparkMax(FEED_MOTOR_ID, MotorType.kBrushless);;
    public final CANSparkMax launch_motor = new CANSparkMax(LAUNCH_MOTOR_ID, MotorType.kBrushless);
    public final RelativeEncoder feed_motor_encoder;
    public final RelativeEncoder launch_motor_encoder;

    public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    public Shooter() {
        feed_motor_encoder = feed_motor.getEncoder();
        launch_motor_encoder = launch_motor.getEncoder();

        feed_motor.setIdleMode(IdleMode.kBrake);
        launch_motor.setIdleMode(IdleMode.kBrake);

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

    @Override public void periodic(){
        SmartDashboard.putNumber("Shooter/LaunchMotorVelocity", launch_motor_encoder.getVelocity());
        SmartDashboard.putNumber("Shooter/LaunchMotorVoltage", launch_motor.getBusVoltage());
    }
}
