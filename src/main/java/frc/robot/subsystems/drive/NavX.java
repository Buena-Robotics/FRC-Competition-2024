package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;

public abstract class NavX extends AHRS {
    @AutoLog public static class NavXInputs {
        public boolean calibrating = false;
        public boolean moving = false;
        public boolean rotating = false;

        public long timestamp = 0;
        public double angle_adjustment = 0.0;
        public double temperature_celcius = 0.0;

        public Rotation2d angle = new Rotation2d();
        public double roll_radians = 0.0;
        public double pitch_radians = 0.0;
        public double yaw_radians = 0.0;

        public double velocity_x = 0.0;
        public double velocity_y = 0.0;
        public double velocity_z = 0.0;

        public double displacement_x = 0.0;
        public double displacement_y = 0.0;
        public double displacement_z = 0.0;
    }

    protected NavXInputsAutoLogged inputs = new NavXInputsAutoLogged();

    public NavX(SerialPort.Port serial_port_id){ 
        super(serial_port_id);
        super.resetDisplacement();
    }

    public abstract void updateInputs();
    public abstract void updateSim();

    public void periodic(){

    }

    public double getRollRadians() { return Units.degreesToRadians(getRoll());  }
    public double getPitchRadians(){ return Units.degreesToRadians(getPitch()); }
    public double getYawRadians()  { return Units.degreesToRadians(getYaw());   }

    public double[] getRotationArrayDegrees(){ return new double[]{getRoll(), getPitch(), getYaw()}; }
    public double[] getRotationArrayRadians(){ return new double[]{getRollRadians(), getPitchRadians(), getYawRadians()}; }

    public double[] getDisplacementMeters(){ return new double[]{getDisplacementX(), getDisplacementY(), getDisplacementZ()}; }
    public double[] getVelocityMetersPerSecond(){ return new double[]{getVelocityX(), getVelocityY(), getVelocityZ()}; }

    public Rotation3d getRotation3d() { return new Rotation3d(getRollRadians(), getPitchRadians(), getYawRadians()); }
    public Translation3d getTranslation3d() { return new Translation3d(getDisplacementX(), getDisplacementY(), getDisplacementZ()); }
    public Pose3d getPose3d(){ return new Pose3d(getTranslation3d(), getRotation3d()); }
}
