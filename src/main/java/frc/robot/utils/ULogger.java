package frc.robot.utils;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public final class ULogger {
    private static LogTable table = new LogTable(Logger.getRealTimestamp()).getSubtable("RealOutputs");
    private static NT4Publisher publisher = new NT4Publisher();

    public static void periodic(){
        publisher.putTable(table);
        table = new LogTable(Logger.getRealTimestamp()).getSubtable("RealOutputs");
    }
    public static void recordOutput(String key, byte[] value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, boolean value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, int value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, long value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, float value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, double value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, String value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static <E extends Enum<E>> void recordOutput(String key, E value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static <U extends Unit<U>> void recordOutput(String key, Measure<U> value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, boolean[] value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, int[] value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, long[] value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, float[] value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, double[] value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, String[] value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static <T> void recordOutput(String key, Struct<T> struct, T value) {
        Logger.recordOutput(key, struct, value);
        table.put(key, struct, value);
    }
    @SuppressWarnings("unchecked")
    public static <T> void recordOutput(String key, Struct<T> struct, T... value) {
        Logger.recordOutput(key, struct, value);
        table.put(key, struct, value);
    }
    public static <T extends WPISerializable> void recordOutput(String key, T value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    @SuppressWarnings("unchecked") 
    public static <T extends StructSerializable> void recordOutput(String key, T... value) {
        Logger.recordOutput(key, value);
        table.put(key, value);
    }
    public static void recordOutput(String key, Mechanism2d value) {
        Logger.recordOutput(key, value);
        value.akitLog(table.getSubtable(key));
    }
}
