package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ShooterReal extends Shooter {
    private static final int SHOOTER_LEDGE_BREAKER_ID = 1;
    private static final int HOLDING_NOTE_BREAKER_ID = 2;
    
    private static final int FEED_MOTOR_ID = 10;
    private static final int LAUNCH_MOTOR_ID = 9;
    private static final int FEED_SMART_CURRENT_LIMIT = 80; 
    private static final int LAUNCH_SMART_CURRENT_LIMIT = 80;

    private final DigitalInput shooter_ledge_beam_breaker = new DigitalInput(SHOOTER_LEDGE_BREAKER_ID);
    private final DigitalInput holding_note_beam_breaker = new DigitalInput(HOLDING_NOTE_BREAKER_ID);

    private final CANSparkMax feed_motor = new CANSparkMax(FEED_MOTOR_ID, MotorType.kBrushless);;
    private final CANSparkMax launch_motor = new CANSparkMax(LAUNCH_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder feed_encoder;
    private final RelativeEncoder launch_encoder;

    public ShooterReal() {
        super();
        
        this.launch_motor.enableVoltageCompensation(12.0);
        this.feed_motor.enableVoltageCompensation(12.0);

        this.feed_encoder = feed_motor.getEncoder();
        this.launch_encoder = launch_motor.getEncoder();

        this.feed_motor.setIdleMode(IdleMode.kBrake);
        this.launch_motor.setIdleMode(IdleMode.kBrake);

        this.feed_motor.setSmartCurrentLimit(FEED_SMART_CURRENT_LIMIT);
        this.launch_motor.setSmartCurrentLimit(LAUNCH_SMART_CURRENT_LIMIT);
    }

    @Override public void updateInputs(){
        //use current pose + climb angle
        inputs.estimated_note_trajectory = new Pose3d[] {};

        inputs.shooter_ledge_beam_broke = shooter_ledge_beam_breaker.get();
        inputs.holding_note_beam_broke = holding_note_beam_breaker.get();

        inputs.feed_velocity_rotations_per_second = feed_encoder.getVelocity();
        inputs.feed_applied_volts = feed_motor.getAppliedOutput() * feed_motor.getBusVoltage();
        inputs.feed_current_amps = new double[] {feed_motor.getOutputCurrent()};
        inputs.feed_temp_celcius = new double[] {feed_motor.getMotorTemperature()};

        inputs.launch_velocity_rotations_per_second = launch_encoder.getVelocity();
        inputs.launch_applied_volts = launch_motor.getAppliedOutput() * launch_motor.getBusVoltage();
        inputs.launch_current_amps = new double[] {launch_motor.getOutputCurrent()};
        inputs.launch_temp_celcius = new double[] {launch_motor.getMotorTemperature()};
    }
    @Override public void setFeedVoltage(double volts){ feed_motor.setVoltage(volts); }
    @Override public void setLaunchVoltage(double volts){ launch_motor.setVoltage(volts); }
}
