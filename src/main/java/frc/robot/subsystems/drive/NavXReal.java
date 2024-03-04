package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;

public class NavXReal extends NavX {
    public NavXReal(SerialPort.Port serial_port_id){ 
        super(serial_port_id); 
        this.resetDisplacement();
    }

    @Override public void updateInputs() {
        inputs.calibrating = super.isCalibrating();
        inputs.moving = super.isMoving();
        inputs.rotating = super.isRotating();

        inputs.timestamp = super.getLastSensorTimestamp();
        inputs.angle_adjustment = super.getAngleAdjustment();
        inputs.temperature_celcius = super.getTempC();

        inputs.angle = super.getRotation2d();
        inputs.angle_rate = super.getRate(); //TODO: Add to this Sim
        inputs.roll_radians = Units.degreesToRadians(super.getRoll());
        inputs.pitch_radians = Units.degreesToRadians(super.getPitch());
        inputs.yaw_radians = Units.degreesToRadians(super.getYaw());

        inputs.velocity_x = super.getVelocityX();
        inputs.velocity_y = super.getVelocityY();
        inputs.velocity_z = super.getVelocityZ();

        inputs.displacement_x = super.getDisplacementX();
        inputs.displacement_y = super.getDisplacementY();
        inputs.displacement_z = super.getDisplacementZ();

        inputs.update_count = super.getUpdateCount();
        inputs.acceleration_full_scale_range_g = super.getAccelFullScaleRangeG();
        inputs.get_gyro_full_scale_range_dps = super.getGyroFullScaleRangeDPS();
        inputs.compass_heading = super.getCompassHeading();
        inputs.fused_heading = super.getFusedHeading();

        inputs.raw_acceleration_x = super.getRawAccelX();
        inputs.raw_acceleration_y = super.getRawAccelY();
        inputs.raw_acceleration_z = super.getRawAccelZ();

        inputs.raw_gyro_x = super.getRawGyroX();
        inputs.raw_gyro_y = super.getRawGyroY();
        inputs.raw_gyro_z = super.getRawGyroZ();
        
        inputs.raw_mag_x = super.getRawMagX();
        inputs.raw_mag_y = super.getRawMagY();
        inputs.raw_mag_z = super.getRawMagZ();

        inputs.world_linear_acceleration_x = super.getWorldLinearAccelX();
        inputs.world_linear_acceleration_y = super.getWorldLinearAccelY();
        inputs.world_linear_acceleration_z = super.getWorldLinearAccelZ();
    }
}
