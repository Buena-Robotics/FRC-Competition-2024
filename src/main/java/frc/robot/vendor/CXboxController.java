// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vendor;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

@SuppressWarnings("MethodName")
public class CXboxController extends CommandGenericHID {
    private final XboxController m_hid;

    public CXboxController(int port) {
        super(port);
        m_hid = new XboxController(port);
    }

    @Override public XboxController getHID() { return m_hid; }

    public CTrigger leftBumper() { return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger leftBumper(EventLoop loop) { return m_hid.leftBumper(loop).castTo(CTrigger::new); }

    public CTrigger rightBumper() { return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger rightBumper(EventLoop loop) { return m_hid.rightBumper(loop).castTo(CTrigger::new); }

    public CTrigger leftStick() { return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger leftStick(EventLoop loop) { return m_hid.leftStick(loop).castTo(CTrigger::new); }

    public CTrigger rightStick() { return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger rightStick(EventLoop loop) { return m_hid.rightStick(loop).castTo(CTrigger::new); }

    public CTrigger a() { return a(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger a(EventLoop loop) { return m_hid.a(loop).castTo(CTrigger::new); }

    public CTrigger b() { return b(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger b(EventLoop loop) { return m_hid.b(loop).castTo(CTrigger::new); }

    public CTrigger x() { return x(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger x(EventLoop loop) { return m_hid.x(loop).castTo(CTrigger::new); }

    public CTrigger y() { return y(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger y(EventLoop loop) { return m_hid.y(loop).castTo(CTrigger::new); }

    public CTrigger start() { return start(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger start(EventLoop loop) { return m_hid.start(loop).castTo(CTrigger::new); }

    public CTrigger back() { return back(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger back(EventLoop loop) { return m_hid.back(loop).castTo(CTrigger::new); }

    public CTrigger leftCTrigger(double threshold, EventLoop loop) { return m_hid.leftTrigger(threshold, loop).castTo(CTrigger::new); }
    public CTrigger leftCTrigger(double threshold) { return leftCTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger leftCTrigger() { return leftCTrigger(0.5); }

    public CTrigger rightCTrigger(double threshold, EventLoop loop) { return m_hid.rightTrigger(threshold, loop).castTo(CTrigger::new); }
    public CTrigger rightCTrigger(double threshold) { return rightCTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public CTrigger rightCTrigger() { return rightCTrigger(0.5); }

    public double getLeftX() { return m_hid.getLeftX(); }
    public double getRightX() { return m_hid.getRightX(); }
    public double getLeftY() { return m_hid.getLeftY(); }
    public double getRightY() { return m_hid.getRightY(); }

    public double getLeftCTriggerAxis() { return m_hid.getLeftTriggerAxis(); }
    public double getRightCTriggerAxis() { return m_hid.getRightTriggerAxis(); }
}
