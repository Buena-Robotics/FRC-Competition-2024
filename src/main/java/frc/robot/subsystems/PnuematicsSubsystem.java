package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PnuematicsSubsystem extends SubsystemBase{
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private PneumaticHub pneumaticsHub = new PneumaticHub();
    
    public PnuematicsSubsystem(){
        
    }
    
}
