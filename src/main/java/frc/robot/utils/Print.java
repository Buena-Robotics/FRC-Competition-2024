package frc.robot.utils;

public final class Print {
    public static void log(String format, Object... args){ print_type("LOG", format, args); }
    public static void warn(String format, Object... args){ print_type("WARN", format, args); }
    public static void error(String format, Object... args){ print_type("ERROR", format, args); }
    private static void print_type(String type, String format, Object... args){
        System.out.println( String.format("[%s]: %s", type, String.format(format, args)) );
    }
}
