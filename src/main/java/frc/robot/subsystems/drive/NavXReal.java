package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.SerialPort;

public class NavXReal extends NavX {
    public NavXReal(SerialPort.Port serial_port_id){ 
        super(serial_port_id); 
        this.resetDisplacement();
    }
    
    @Override public void updateInputs() {
        
    }
    @Override public void updateSim() {}
}
