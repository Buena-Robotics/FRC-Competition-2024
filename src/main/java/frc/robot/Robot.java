// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IO;

public class Robot extends LoggedRobot {
    private Command autonomous_command;

    private RobotContainer robot_container;

    public Robot(){
        super(Robot.defaultPeriodSecs);
    }
    @Override public void robotInit() {
        // Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter("/AdvantageKit-ReplayLogs/Simulation/"));
        // Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance(10, ModuleType.kRev);
        Logger.start();

        robot_container = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override public void disabledInit() {}

    @Override public void disabledPeriodic() {}

    @Override public void autonomousInit() {
        autonomous_command = robot_container.getAutonomousCommand();

        if (autonomous_command != null)
        autonomous_command.schedule();
    }

    @Override public void autonomousPeriodic() {}

    @Override public void teleopInit() {
        if (autonomous_command != null)
            autonomous_command.cancel();
    }

    @Override public void teleopPeriodic() {
        IO.controller.setRumble(RumbleType.kLeftRumble, IO.controller.getRightTriggerAxis());
    }

    @Override public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override public void testPeriodic() {}

    @Override public void simulationInit() {}

    @Override public void simulationPeriodic() {}
}
