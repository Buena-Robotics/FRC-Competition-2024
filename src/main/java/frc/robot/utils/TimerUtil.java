package frc.robot.utils;

public class TimerUtil {

    public long lastMS = System.currentTimeMillis();
    
    public void reset() {
        lastMS = System.currentTimeMillis();
    }
    
    public boolean hasTimeElapsed(long time_ms, boolean reset) {
        if (lastMS > System.currentTimeMillis())
            lastMS = System.currentTimeMillis();
        if (System.currentTimeMillis()-lastMS > time_ms) {
            if (reset) reset();
            return true;
        } else return false;
    }
}
