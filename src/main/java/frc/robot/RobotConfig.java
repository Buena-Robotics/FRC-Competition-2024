package frc.robot;

import frc.robot.utils.Print;

public final class RobotConfig {
    public static final boolean LOG_SIMULATION_TO_FILE = false;
    public static final boolean REPLAY = true;
    public static final boolean FORCE_BLUE_ALLIANCE = true;

    public static enum RobotMode {
        SIM,
        REAL,
        REPLAY
    }
    public static RobotMode getRobotMode(){
        if(Robot.isReal()) return RobotMode.REAL;
        else if(Robot.isSimulation() && !REPLAY) return RobotMode.SIM;
        Print.log("REPLAY MODE");
        return RobotMode.REPLAY;
    }
}
