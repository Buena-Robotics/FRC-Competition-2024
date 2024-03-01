package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SerialPort;

public abstract class NavX extends AHRS {
    @AutoLog public static class NavXInputs {
        public boolean calibrating = false;
        public boolean moving = false;
        public boolean rotating = false;

        public long timestamp = 0;
        public double delta_time = 0;
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

        public Pose2d estimated_pose = new Pose2d();
    }

    protected NavXInputsAutoLogged inputs = new NavXInputsAutoLogged();
    protected Pose2d start_pose = new Pose2d();

    public NavX(SerialPort.Port serial_port_id, Pose2d start_pose){
        this(serial_port_id);
        this.start_pose = start_pose;
    }

    public NavX(SerialPort.Port serial_port_id){ 
        super(serial_port_id);
        super.resetDisplacement();
    }

    public abstract void updateInputs();

    @Override public void periodic(){
        updateInputs();
        Logger.processInputs("Drive/NavX", inputs);
    }

    protected double removeNoise(double value){
        return removeNoise(value, 5);
    }
    protected double removeNoise(double value, int precision){
        double move_decimal = Math.pow(10, precision);
        return (double)((int)(value*move_decimal)) / move_decimal;
    }

    public void setStartPose(Pose2d start_pose){ this.start_pose = start_pose; }

    public double getRollRadians() { return inputs.roll_radians;  }
    public double getPitchRadians(){ return inputs.pitch_radians; }
    public double getYawRadians()  { return inputs.yaw_radians;   }

    public double[] getDisplacementMeters(){ return new double[]{inputs.displacement_x, inputs.displacement_y, inputs.displacement_z}; }
    public double[] getVelocityMetersPerSecond(){ return new double[]{inputs.velocity_x, inputs.velocity_y, inputs.velocity_z}; }

    public Rotation3d getRotation3d() { return new Rotation3d(getRollRadians(), getPitchRadians(), getYawRadians()); }
    public Translation3d getTranslation3d() { return new Translation3d(inputs.velocity_x, inputs.velocity_y, inputs.velocity_z); }
    public Pose3d getPose3d(){ return new Pose3d(getTranslation3d(), getRotation3d()).relativeTo(new Pose3d(start_pose)); }
    public Pose2d getEstimatedPose(){ return inputs.estimated_pose; }
}
