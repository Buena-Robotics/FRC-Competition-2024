package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModuleReal extends SwerveModule {
    private final double DRIVE_ENCODER_REDUCTION = 1; // (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double TURN_ENCODER_REDUCTION = 1;//150.0 / 7.0;

    private final CANSparkMax drive_motor;
    private final CANSparkMax turn_motor;
    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;
    private final AnalogInput absolute_encoder;
    private final Rotation2d absolute_encoder_offset_radians;

    public SwerveModuleReal(String module_name, int drive_motor_id, int turn_motor_id, int absolute_encoder_id, Rotation2d absolute_encoder_offset_radians) { 
        super(module_name, absolute_encoder_id);
        
        this.drive_motor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        this.turn_motor =  new CANSparkMax(turn_motor_id,  MotorType.kBrushless);

        this.drive_encoder = drive_motor.getEncoder();
        this.drive_encoder.setPositionConversionFactor(DRIVE_ENCODER_ROTATION_TO_METERS);
        this.drive_encoder.setVelocityConversionFactor(DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND);
        
        this.turn_encoder = turn_motor.getEncoder();
        this.turn_encoder.setPositionConversionFactor(TURN_ENCODER_ROTATION_TO_RADIANS);
        this.turn_encoder.setVelocityConversionFactor(TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND);

        this.absolute_encoder = new AnalogInput(absolute_encoder_id);
        this.absolute_encoder_offset_radians = absolute_encoder_offset_radians;
        
        this.drive_motor.enableVoltageCompensation(12.0);
        this.turn_motor.enableVoltageCompensation(12.0);
        
        this.drive_encoder.setPosition(0.0);
        this.drive_encoder.setMeasurementPeriod(10);
        this.drive_encoder.setAverageDepth(2);
    
        this.turn_encoder.setPosition(getAbsoluteEncoderRadians());
        this.turn_encoder.setMeasurementPeriod(10);
        this.turn_encoder.setAverageDepth(2);

        checkConnections();
    }

    private void checkConnections(){
        REVLibError drive_revlib_error = drive_motor.getLastError();
        REVLibError turn_revlib_error = drive_motor.getLastError();
        boolean absolute_encoder_connected = absolute_encoder.isAccumulatorChannel();

        String str_error = "";

        if(drive_revlib_error != REVLibError.kOk)
            str_error += String.format("[SModule%d: %s - Drive Motor]: %s", index, module_name, SparkMaxUtils.REVErrorToString(drive_revlib_error));
        if(turn_revlib_error != REVLibError.kOk)
            str_error += String.format("[SModule%d: %s - Turn Motor]: %s", index, module_name, SparkMaxUtils.REVErrorToString(turn_revlib_error));
        if(!absolute_encoder_connected)
            str_error += String.format("[SModule%d: %s - Abs Encoder]: Disconnected", index, module_name);
        
        if(!str_error.equals(""))
            DriverStation.reportError(str_error, true);
    }

    private double getAbsoluteEncoderRadians(){ 
        double angle = absolute_encoder.getVoltage() / RobotController.getVoltage5V();
        Rotation2d rotation = new Rotation2d(angle * 2.0 * Math.PI);
        return rotation.minus(absolute_encoder_offset_radians).getRadians();
    }

    @Override public void updateInputs() {
        inputs.drive_position_radians =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.drive_position_radians,
                Units.rotationsToRadians(drive_encoder.getPosition()) / DRIVE_ENCODER_REDUCTION);
        inputs.drive_velocity_radians_per_second =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.drive_velocity_radians_per_second,
                Units.rotationsPerMinuteToRadiansPerSecond(drive_encoder.getVelocity())
                    / DRIVE_ENCODER_REDUCTION);
        inputs.drive_applied_volts = drive_motor.getAppliedOutput() * drive_motor.getBusVoltage();
        inputs.drive_current_amps = new double[] {drive_motor.getOutputCurrent()};
        inputs.drive_temp_celcius = new double[] {drive_motor.getMotorTemperature()};

        inputs.turn_absolute_position_radians =
            MathUtil.angleModulus(
                new Rotation2d(
                        absolute_encoder.getVoltage()
                            / RobotController.getVoltage5V()
                            * 2.0
                            * Math.PI)
                    .minus(absolute_encoder_offset_radians)
                    .getRadians());
        inputs.turn_position_radians =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.turn_position_radians,
                Units.rotationsToRadians(turn_encoder.getPosition())
                    / TURN_ENCODER_REDUCTION);
        inputs.turn_velocity_radians_per_second =
            SparkMaxUtils.cleanSparkMaxValue(
                inputs.turn_velocity_radians_per_second,
                Units.rotationsPerMinuteToRadiansPerSecond(turn_encoder.getVelocity())
                    / TURN_ENCODER_REDUCTION);
        inputs.turn_applied_volts = turn_motor.getAppliedOutput() * turn_motor.getBusVoltage();
        inputs.turn_current_amps = new double[] {turn_motor.getOutputCurrent()};
        inputs.turn_temp_celcius = new double[] {turn_motor.getMotorTemperature()};
    }

    @Override public void setDriveVoltage(double volts) { drive_motor.setVoltage(volts); }
    @Override public void setTurnVoltage (double volts) { turn_motor.setVoltage(volts); }
    @Override public void setDriveBrakeMode(boolean enable) { drive_motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast); }
    @Override public void setTurnBrakeMode (boolean enable) { turn_motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast); }
}