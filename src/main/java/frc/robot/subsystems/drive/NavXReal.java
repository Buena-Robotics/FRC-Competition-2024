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

        inputs.angle = super.getRotation2d();
        inputs.angle_rate = super.getRate();
        inputs.roll_radians = Units.degreesToRadians(super.getRoll());
        inputs.pitch_radians = Units.degreesToRadians(super.getPitch());
        inputs.yaw_radians = Units.degreesToRadians(super.getYaw());

        inputs.velocity_x = super.getVelocityX();
        inputs.velocity_y = super.getVelocityY();
        inputs.velocity_z = super.getVelocityZ();

        inputs.displacement_x = super.getDisplacementX();
        inputs.displacement_y = super.getDisplacementY();
        inputs.displacement_z = super.getDisplacementZ();
    }
}
