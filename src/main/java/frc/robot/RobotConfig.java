package frc.robot;

public final class RobotConfig {
    public static final boolean LOG_SIMULATION_TO_FILE = false;
    public static final boolean REPLAY = false;
    public static final boolean FEATURE_UAUTO = true;    

    public static enum RobotMode {
        SIM,
        REAL,
        REPLAY
    }
    public static RobotMode getRobotMode(){
        if(Robot.isReal()) return RobotMode.REAL;
        else if(Robot.isSimulation() && !REPLAY) return RobotMode.SIM;
        return RobotMode.REPLAY;
    }
}
