package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModuleReal extends SwerveModule {
    private static final int CAN_TIMEOUT = 500;

    private final CANSparkMax drive_motor;
    private final CANSparkMax turn_motor;
    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;
    private final AnalogInput absolute_encoder;
    private final double absolute_encoder_offset_radians;

    public SwerveModuleReal(String module_name, int drive_motor_id, int turn_motor_id, int absolute_encoder_id, double absolute_encoder_offset_radians) { 
        super(module_name, absolute_encoder_id);
        
        this.absolute_encoder = new AnalogInput(absolute_encoder_id);
        this.absolute_encoder_offset_radians = absolute_encoder_offset_radians;

        this.drive_motor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        this.drive_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        this.drive_motor.enableVoltageCompensation(12.0);
        this.drive_motor.setCANTimeout(CAN_TIMEOUT);
        
        this.turn_motor =  new CANSparkMax(turn_motor_id,  MotorType.kBrushless);
        this.turn_motor.enableVoltageCompensation(12.0);
        this.turn_motor.setCANTimeout(CAN_TIMEOUT);

        this.drive_encoder = drive_motor.getEncoder();
        this.drive_encoder.setPosition(0.0);
        this.drive_encoder.setMeasurementPeriod(10);
        this.drive_encoder.setAverageDepth(2);
        
        this.turn_encoder = turn_motor.getEncoder();
        this.turn_encoder.setPosition(0);
        this.turn_encoder.setMeasurementPeriod(10);
        this.turn_encoder.setAverageDepth(2);

        this.drive_motor.setCANTimeout(0);
        this.turn_motor.setCANTimeout(0);
    }

    @Override public void updateInputs() {
        inputs.drive_position_radians =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.drive_position_radians,
                Units.rotationsToRadians(drive_encoder.getPosition()) * DRIVE_ENCODER_ROTATION_TO_METERS); //TODO: Replace '/' with '*'
        inputs.drive_velocity_radians_per_second =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.drive_velocity_radians_per_second,
                Units.rotationsPerMinuteToRadiansPerSecond(drive_encoder.getVelocity()) * DRIVE_ENCODER_ROTATION_TO_METERS);
        inputs.drive_applied_volts = drive_motor.getAppliedOutput() * drive_motor.getBusVoltage();
        inputs.drive_current_amps = new double[] {drive_motor.getOutputCurrent()};
        inputs.drive_temp_celcius = new double[] {drive_motor.getMotorTemperature()};

        inputs.turn_absolute_position_radians =
            MathUtil.angleModulus(
                new Rotation2d(
                        (absolute_encoder.getVoltage()
                            / RobotController.getVoltage5V()
                            * 2.0
                            * Math.PI - absolute_encoder_offset_radians))
                    .getRadians());
        inputs.turn_position_radians =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.turn_position_radians,
                Units.rotationsToRadians(turn_encoder.getPosition()) * TURN_ENCODER_ROTATION_TO_RADIANS);
        inputs.turn_velocity_radians_per_second =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.turn_velocity_radians_per_second,
                Units.rotationsPerMinuteToRadiansPerSecond(turn_encoder.getVelocity()) * TURN_ENCODER_ROTATION_TO_RADIANS);
        inputs.turn_applied_volts = turn_motor.getAppliedOutput() * turn_motor.getBusVoltage();
        inputs.turn_current_amps = new double[] {turn_motor.getOutputCurrent()};
        inputs.turn_temp_celcius = new double[] {turn_motor.getMotorTemperature()};
    }

    @Override public void setDriveVoltage(double volts) { drive_motor.setVoltage(volts); }
    @Override public void setTurnVoltage (double volts) { turn_motor.setVoltage(volts); }
    @Override public void setDriveBrakeMode(boolean enable) { drive_motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast); }
    @Override public void setTurnBrakeMode (boolean enable) { turn_motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast); }

    // public double absEncoder(){ return absolute_encoder.getVoltage() / RobotController.getVoltage5V(); }
}