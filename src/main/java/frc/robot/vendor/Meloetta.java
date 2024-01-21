package frc.robot.vendor;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

public final class Meloetta {
    public static final class PI {
        // public static Pose3d getPose(){}
    }

    private static final Map<String, Sendable> tables_to_data = new HashMap<>();

    private static final NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
    private static final NetworkTable nt_meloetta_table = nt_inst.getTable("Meloetta");

    public static boolean putNumber(String key, double value){ return nt_meloetta_table.getEntry(key).setDouble(value); }
    public static double getNumber(String key, double default_value){ return nt_meloetta_table.getEntry(key).getDouble(default_value); }
    
    public static boolean putString(String key, String value){ return nt_meloetta_table.getEntry(key).setString(value); }
    public static String getString(String key, String default_value){ return nt_meloetta_table.getEntry(key).getString(default_value); }

    public static void putData(String key, Sendable data){
        Sendable sddata = tables_to_data.get(key);
        if (sddata == null || sddata != data) {
            tables_to_data.put(key, data);
            NetworkTable dataTable = nt_meloetta_table.getSubTable(key);
            SendableBuilderImpl builder = new SendableBuilderImpl();
            builder.setTable(dataTable);
            SendableRegistry.publish(data, builder);
            builder.startListeners();
            dataTable.getEntry(".name").setString(key);
        }
    }

    // TODO Implement CommandRunType
    public static enum CommandRunType {
        kWhileHolding,
        KRunOnce
    }
    public static void putCommand(String key, Sendable command, CommandRunType command_run_type){
        // TODO Implement Meloetta.putCommand(...)
    }
}
