package frc.robot.subsystems.notearm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class NoteArmSim extends NoteArm {
    private final Mechanism2d color_sensor_mechanism = new Mechanism2d(10, 10, new Color8Bit());
    
    public NoteArmSim(){
        super();
        SmartDashboard.putData("NoteArm/ColorSensor", color_sensor_mechanism);
    }

    @Override public void periodic() {
        super.periodic();
        color_sensor_mechanism.setBackgroundColor(new Color8Bit(super.getColor()));        
    }
}
