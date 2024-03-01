package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class NavXReal extends NavX {
    private Pose2d estimated_pose = new Pose2d();
    private double last_timestamp = Timer.getFPGATimestamp();
    private double delta_time = 0;

    public NavXReal(SerialPort.Port serial_port_id){ 
        super(serial_port_id); 
        this.resetDisplacement();
    }
    
    private Pose2d updatePose(){
        estimated_pose.transformBy(
            new Transform2d(
                inputs.velocity_x * delta_time, 
                inputs.velocity_y * delta_time, 
                Rotation2d.fromDegrees(inputs.angle_rate * delta_time)));

        double timestamp = Timer.getFPGATimestamp();
        delta_time = timestamp - last_timestamp; 
        last_timestamp = timestamp;
        return estimated_pose.relativeTo(start_pose);
    }

    @Override public void updateInputs() {
        inputs.calibrating = super.isCalibrating();
        inputs.moving = super.isMoving();
        inputs.rotating = super.isRotating();

        inputs.timestamp = super.getLastSensorTimestamp();
        inputs.angle_adjustment = super.getAngleAdjustment();
        inputs.temperature_celcius = super.getTempC();

        inputs.angle = super.getRotation2d();
        inputs.angle_rate = removeNoise(super.getRate()); //TODO: Add to this Sim
        inputs.roll_radians = removeNoise(Units.degreesToRadians(super.getRoll()));
        inputs.pitch_radians = removeNoise(Units.degreesToRadians(super.getPitch()));
        inputs.yaw_radians = removeNoise(Units.degreesToRadians(super.getYaw()));

        inputs.velocity_x = removeNoise(super.getVelocityX());
        inputs.velocity_y = removeNoise(super.getVelocityY());
        inputs.velocity_z = removeNoise(super.getVelocityZ());

        inputs.displacement_x = removeNoise(super.getDisplacementX());
        inputs.displacement_y = removeNoise(super.getDisplacementY());
        inputs.displacement_z = removeNoise(super.getDisplacementZ());

        inputs.estimated_pose = updatePose();
    }
}
