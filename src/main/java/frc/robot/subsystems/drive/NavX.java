package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public abstract class NavX extends AHRS {
    @AutoLog public static class NavXInputs {
        public boolean calibrating = false;
        public boolean moving = false;
        public boolean rotating = false;

        public long timestamp = 0;
        public double angle_adjustment = 0.0;
        public double temperature_celcius = 0.0;

        public Rotation2d angle = new Rotation2d();
        public double angle_rate = 0.0;
        public double roll_radians = 0.0;
        public double pitch_radians = 0.0;
        public double yaw_radians = 0.0;

        public double velocity_x = 0.0;
        public double velocity_y = 0.0;
        public double velocity_z = 0.0;

        public double displacement_x = 0.0;
        public double displacement_y = 0.0;
        public double displacement_z = 0.0;

        public double update_count = 0.0;
        public int acceleration_full_scale_range_g = 0;
        public int get_gyro_full_scale_range_dps = 0;
        public float compass_heading = 0.0f;
        public float fused_heading = 0.0f;

        public float raw_acceleration_x = 0.0f;
        public float raw_acceleration_y = 0.0f;
        public float raw_acceleration_z = 0.0f;

        public float raw_gyro_x = 0.0f;
        public float raw_gyro_y = 0.0f;
        public float raw_gyro_z = 0.0f;
        
        public float raw_mag_x = 0.0f;
        public float raw_mag_y = 0.0f;
        public float raw_mag_z = 0.0f;

        public float world_linear_acceleration_x = 0.0f;
        public float world_linear_acceleration_y = 0.0f;
        public float world_linear_acceleration_z = 0.0f;
        }

    private Pose2d estimated_pose = new Pose2d();
    protected double last_timestamp = Timer.getFPGATimestamp();
    protected double delta_time = 0;
    protected NavXInputsAutoLogged inputs = new NavXInputsAutoLogged();
    protected Pose2d start_pose = new Pose2d();
    protected Rotation2d field_oriented_heading = new Rotation2d();

    public NavX(SerialPort.Port serial_port_id, Pose2d start_pose){
        this(serial_port_id);
        this.start_pose = start_pose;
    }

    public NavX(SerialPort.Port serial_port_id){ 
        super(serial_port_id);
        super.resetDisplacement();
    }

    public abstract void updateInputs();

    public void periodic(){
        updateInputs();
        Logger.processInputs("Drive/NavX", inputs);
        
        updateDeltaTime();
        Logger.recordOutput("NavX/EstimatedPose", updatePose());
    }

    private void updateDeltaTime(){
        double timestamp = Timer.getFPGATimestamp();
        delta_time = timestamp - last_timestamp; 
        last_timestamp = timestamp;
    }
    public double getDeltaTime(){ return delta_time; }

    private Pose2d updatePose(){
        estimated_pose = estimated_pose.transformBy(
            new Transform2d(
                inputs.velocity_x * getDeltaTime(), 
                0, 
                Rotation2d.fromDegrees(inputs.angle_rate * delta_time)));

        return estimated_pose.relativeTo(start_pose);
    }

    protected double removeNoise(double value){
        return removeNoise(value, 5);
    }
    protected double removeNoise(double value, int precision){
        double move_decimal = Math.pow(10, precision);
        return (double)((int)(value*move_decimal)) / move_decimal;
    }

    public void setStartPose(Pose2d start_pose){ this.start_pose = start_pose; }
    public void setFieldOrientedHeading(Rotation2d heading){ this.field_oriented_heading = heading; }

    public Rotation2d getFieldOrientedHeading(){ return this.field_oriented_heading; }

    public double getRollRadians() { return inputs.roll_radians;  }
    public double getPitchRadians(){ return inputs.pitch_radians; }
    public double getYawRadians()  { return inputs.yaw_radians;   }

    public double[] getDisplacementMeters(){ return new double[]{inputs.displacement_x, inputs.displacement_y, inputs.displacement_z}; }
    public double[] getVelocityMetersPerSecond(){ return new double[]{inputs.velocity_x, inputs.velocity_y, inputs.velocity_z}; }

    public Rotation3d getRotation3d() { return new Rotation3d(getRollRadians(), getPitchRadians(), getYawRadians()); }
    public Translation3d getTranslation3d() { return new Translation3d(inputs.velocity_x, inputs.velocity_y, inputs.velocity_z); }
    public Pose3d getPose3d(){ return new Pose3d(getTranslation3d(), getRotation3d()).relativeTo(new Pose3d(start_pose)); }
    public Pose2d getEstimatedPose(){ return estimated_pose; }
}
