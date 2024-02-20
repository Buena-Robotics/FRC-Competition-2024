package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private static final String TABLE_KEY = "Tunable";
    private final String key;
    
    public TunableNumber(String item_key, double default_value) {
        this.key = TABLE_KEY + "/" + item_key;        
        SmartDashboard.putNumber(this.key, default_value);
    }

    public TunableNumber(String key) { this(key, 0.0); }

    public double get() { return SmartDashboard.getNumber(this.key, 0.0); }
}
