package frc.robot.devices;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;

public class NavX extends AHRS {
    private static final int MINIMUM_AIR_HEIGHT_INCHES = 4;

    public NavX(SPI.Port serial_port_id){ 
        super(serial_port_id); 
        this.resetDisplacement(); 
        this.isConnected();
    }
    
    public boolean isInAir(){ return this.getDisplacementY() > Units.inchesToMeters(MINIMUM_AIR_HEIGHT_INCHES); }

    public Rotation3d getRotation3d() { return new Rotation3d(this.getRoll(), this.getPitch(), this.getYaw()); }
    public Translation3d getTranslation3d() { return new Translation3d(this.getDisplacementX(), this.getDisplacementY(), this.getDisplacementZ()); }
    public Pose3d getPose3d(){ return new Pose3d(getTranslation3d(), getRotation3d()); }

    @Override public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
