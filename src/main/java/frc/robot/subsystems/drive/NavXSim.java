package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.SerialPort;

public class NavXSim extends NavX {
    public NavXSim(SerialPort.Port serial_port_id){
        super(serial_port_id);
    }

    @Override public void updateInputs() {
        
    }
    @Override public void updateSim() {

    }
    // public void updateSimulationAngle(Rotation2d rotation){
    //     if(Robot.isSimulation()) {
    //         simulation_rotation = simulation_rotation.minus(rotation);
    //         this.setAngleAdjustment(simulation_rotation.getDegrees() * 2); 
    //     }
    // }
}
