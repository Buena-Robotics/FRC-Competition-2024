package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public final class RobotConfig {
    public static final boolean LOG_SIMULATION_TO_FILE = false;
    public static final boolean REPLAY = false;
    public static final boolean FORCE_BLUE_ALLIANCE = false && !DriverStation.isFMSAttached();

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
