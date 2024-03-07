package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.TimerUtil;

public class RumbleFeedback extends Command {
    private final TimerUtil timer;
    private final XboxController xbox_controller;
    private final RumbleType rumble_type;
    private final double strength;
    private final long time_ms;
    
    // If time_ms == -1 then it lasts till command is unscheduled
    public RumbleFeedback(XboxController xbox_controller, RumbleType rumble_type, long time_ms){ this(xbox_controller, rumble_type, 0.25, time_ms); }
    public RumbleFeedback(XboxController xbox_controller, RumbleType rumble_type, double strength){ this(xbox_controller, rumble_type, strength, 40); }
    public RumbleFeedback(XboxController xbox_controller, RumbleType rumble_type, double strength, long time_ms){
        this.timer = new TimerUtil();
        this.xbox_controller = xbox_controller;
        this.rumble_type = rumble_type;
        this.strength = strength;
        this.time_ms = time_ms;
    }

    @Override public void initialize() { timer.reset(); xbox_controller.setRumble(rumble_type, strength); }
    @Override public void end(boolean interrupted) { xbox_controller.setRumble(RumbleType.kBothRumble, 0); }
    @Override public boolean isFinished() { return timer.hasTimeElapsed(time_ms, true); }
}
