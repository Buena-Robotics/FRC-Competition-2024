package frc.robot.vendor;

import java.util.Vector;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public final class Meloetta {
    public static final NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
    public static final NetworkTable nt_meloetta_table = nt_inst.getTable("Meloetta");

    public static void putButton(String key, Command command){ }
    public static void putNumber(String key, double default_value){ }
    public static void getNumber(String key, double default_value){ }
    public static void putSwerveDriveSubsystem(SwerveDriveSubsystem sub){ }
}
