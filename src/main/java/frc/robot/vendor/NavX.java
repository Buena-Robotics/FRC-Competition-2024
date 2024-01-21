package frc.robot.vendor;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;

public class NavX extends AHRS {
    private static final int MINIMUM_AIR_HEIGHT_INCHES = 4;

    public NavX(I2C.Port serial_port_id){ 
        super(serial_port_id); 
        this.resetDisplacement(); 
    }
    
    public boolean isInAir(){ return this.getDisplacementY() > Units.inchesToMeters(MINIMUM_AIR_HEIGHT_INCHES); }

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

    @Override public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.publishConstBoolean("Connected?", isConnected());
        builder.addBooleanProperty("Calibrating?", this::isCalibrating, null);
        builder.addDoubleArrayProperty("Rotation Deg", this::getRotationArrayDegrees, null);
        builder.addDoubleArrayProperty("Rotation Rad", this::getRotationArrayRadians, null);
        builder.addDoubleArrayProperty("Displacement Meters", this::getDisplacementMeters, null);
        builder.addDoubleArrayProperty("Velocity Meters|Sec", this::getVelocityMetersPerSecond, null);
    }
}
