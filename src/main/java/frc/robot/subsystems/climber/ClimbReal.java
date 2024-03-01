package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimbReal extends Climb {
    private static final int WINCH_MOTOR_ID = 12;
    private static final int BORE_ENCODER_CHANNEL = 0;

    private final CANSparkMax winch_motor = new CANSparkMax(WINCH_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder winch_encoder = winch_motor.getEncoder();
    private final DutyCycleEncoder bore_encoder = new DutyCycleEncoder(BORE_ENCODER_CHANNEL);
    private final SparkLimitSwitch winch_top_limit_switch;
    private final SparkLimitSwitch winch_bottom_limit_switch;
    public ClimbReal() {
        super();
        this.winch_top_limit_switch = this.winch_motor.getForwardLimitSwitch(Type.kNormallyOpen);
        this.winch_bottom_limit_switch = this.winch_motor.getReverseLimitSwitch(Type.kNormallyOpen);

        this.winch_motor.setIdleMode(IdleMode.kBrake);        
        this.winch_motor.enableVoltageCompensation(12.0);
        this.winch_encoder.setPositionConversionFactor(WINCH_MOTOR_GEAR_RATIO);
        this.winch_encoder.setVelocityConversionFactor(WINCH_MOTOR_GEAR_RATIO);
        this.winch_encoder.setPosition(boreRadiansToWinchRevolutions());
    }

    private double boreRadiansToWinchRevolutions(){
        return 0.0;
    }

    @Override public void updateInputs() {
        inputs.bore_absolute_position_raw = bore_encoder.getAbsolutePosition();
        inputs.bore_absolute_position_radians = ((41*Math.PI)/20) * (bore_encoder.getAbsolutePosition() - 0.146338);
        
        inputs.winch_top_limit_switch_triggered = winch_top_limit_switch.isPressed();
        inputs.winch_bottom_limit_switch_triggered = winch_bottom_limit_switch.isPressed();

        inputs.winch_position_radians = winch_encoder.getPosition() * WINCH_ENCODER_ROTATIONS_TO_RADIANS;
        inputs.winch_velocity_radians_per_second = winch_encoder.getVelocity() * WINCH_ENCODER_ROTATIONS_TO_RADIANS / 60;
        inputs.winch_rotations = winch_encoder.getPosition();
        inputs.winch_velocity_rotations_per_second = winch_encoder.getVelocity() / 60;
        inputs.winch_applied_volts = winch_motor.getAppliedOutput() * winch_motor.getBusVoltage();
        inputs.winch_current_amps = new double[] {winch_motor.getOutputCurrent()};
        inputs.winch_temp_celcius = new double[] {};
    }
    @Override public void setWinchVoltage(double volts) { winch_motor.setVoltage(volts); }

    @Override public void periodic() {
        super.periodic();
        winch_motor.clearFaults();
    }
}
