    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot.vendor;

    import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

    import edu.wpi.first.math.filter.Debouncer;
    import edu.wpi.first.wpilibj.event.EventLoop;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.TimerUtil;

import java.util.function.BooleanSupplier;

    public class CTrigger implements BooleanSupplier {
    private final BooleanSupplier m_condition;
    private final EventLoop m_loop;


    public CTrigger(EventLoop loop, BooleanSupplier condition) {
        m_loop = requireNonNullParam(loop, "loop", "CTrigger");
        m_condition = requireNonNullParam(condition, "condition", "CTrigger");
    }

    public CTrigger(BooleanSupplier condition) {
        this(CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
    }

    // If duration_ms <= 0 then the command will run only once
    public CTrigger onPress(Command command, long duration_ms){
        requireNonNullParam(command, "command", "onPress");
        m_loop.bind(
            new Runnable() {
            TimerUtil length_timer = new TimerUtil();
            boolean was_pressed = m_condition.getAsBoolean();
            boolean running = false;
            boolean mutex = false;
            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();
                if(!running) length_timer.reset();
                if(pressed) was_pressed = true;
                boolean has_released = was_pressed && !pressed;
                boolean ran = duration_ms <= 0 || running && length_timer.hasTimeElapsed(duration_ms, true);
                if ((!mutex || running) && (pressed || running)) {
                    was_pressed = false;
                    running = true;
                    mutex = true;
                    command.schedule();
                    if(has_released) mutex = false;
                    if(ran) {running = false; command.cancel();}
                }
                if(!ran && has_released) mutex = false; 
            }
        });
        return this;
    }

    // If duration_ms <= 0 then the command will run only once
    public CTrigger onRelease(Command command, long duration_ms){
        requireNonNullParam(command, "command", "onRelease");
        m_loop.bind(
            new Runnable() {
            TimerUtil length_timer = new TimerUtil();
            boolean was_pressed = m_condition.getAsBoolean();
            boolean running = false;
            
            @Override public void run() {
                boolean pressed = m_condition.getAsBoolean();
                if(pressed) was_pressed = true;
                if(!running) length_timer.reset();
                boolean ran = duration_ms <= 0 || running && length_timer.hasTimeElapsed(duration_ms, true);
                if(was_pressed && !pressed || running){
                    running = true;
                    was_pressed = false;
                    command.schedule();
                    if(ran) running = false;
                }
            }
        });

        return this;
    }

    public CTrigger onTrue(Command command) {
        requireNonNullParam(command, "command", "onTrue");
        m_loop.bind(
            new Runnable() {
            private boolean m_pressedLast = m_condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();

                if (!m_pressedLast && pressed) {
                command.schedule();
                }

                m_pressedLast = pressed;
            }
            });
        return this;
    }

    public CTrigger onFalse(Command command) {
        requireNonNullParam(command, "command", "onFalse");
        m_loop.bind(
            new Runnable() {
            private boolean m_pressedLast = m_condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();

                if (m_pressedLast && !pressed) {
                command.schedule();
                }

                m_pressedLast = pressed;
            }
            });
        return this;
    }

    public CTrigger whileTrue(Command command) {
        requireNonNullParam(command, "command", "whileTrue");
        m_loop.bind(
            new Runnable() {
            private boolean m_pressedLast = m_condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();

                if (!m_pressedLast && pressed) {
                command.schedule();
                } else if (m_pressedLast && !pressed) {
                command.cancel();
                }

                m_pressedLast = pressed;
            }
            });
        return this;
    }

    public CTrigger whileFalse(Command command) {
        requireNonNullParam(command, "command", "whileFalse");
        m_loop.bind(
            new Runnable() {
            private boolean m_pressedLast = m_condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();

                if (m_pressedLast && !pressed) {
                command.schedule();
                } else if (!m_pressedLast && pressed) {
                command.cancel();
                }

                m_pressedLast = pressed;
            }
            });
        return this;
    }

    public CTrigger toggleOnTrue(Command command) {
        requireNonNullParam(command, "command", "toggleOnTrue");
        m_loop.bind(
            new Runnable() {
            private boolean m_pressedLast = m_condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();

                if (!m_pressedLast && pressed) {
                if (command.isScheduled()) {
                    command.cancel();
                } else {
                    command.schedule();
                }
                }

                m_pressedLast = pressed;
            }
            });
        return this;
    }

    public CTrigger toggleOnFalse(Command command) {
        requireNonNullParam(command, "command", "toggleOnFalse");
        m_loop.bind(
            new Runnable() {
            private boolean m_pressedLast = m_condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();

                if (m_pressedLast && !pressed) {
                if (command.isScheduled()) {
                    command.cancel();
                } else {
                    command.schedule();
                }
                }

                m_pressedLast = pressed;
            }
            });
        return this;
    }

    @Override public boolean getAsBoolean() {
        return m_condition.getAsBoolean();
    }

    public CTrigger and(BooleanSupplier trigger) {
        return new CTrigger(() -> m_condition.getAsBoolean() && trigger.getAsBoolean());
    }

    public CTrigger or(BooleanSupplier trigger) {
        return new CTrigger(() -> m_condition.getAsBoolean() || trigger.getAsBoolean());
    }

    public CTrigger negate() {
        return new CTrigger(() -> !m_condition.getAsBoolean());
    }

    public CTrigger debounce(double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    public CTrigger debounce(double seconds, Debouncer.DebounceType type) {
        return new CTrigger(
            new BooleanSupplier() {
            final Debouncer m_debouncer = new Debouncer(seconds, type);

            @Override
            public boolean getAsBoolean() {
                return m_debouncer.calculate(m_condition.getAsBoolean());
            }
            });
    }
    }
