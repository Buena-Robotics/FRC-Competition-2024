// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SubSystems;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.Print;

public class Robot extends LoggedRobot {
    private RobotContainer robot_container;
    private Command autonomous_command;
    private boolean enabled_flag = false;

    public Robot(){
        super(Robot.defaultPeriodSecs);
    }
    @Override public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        

        switch (RobotConfig.getRobotMode()) {
        case REAL:
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
            Logger.addDataReceiver(new NT4Publisher());
            break;
        case SIM: 
            if(RobotConfig.LOG_SIMULATION_TO_FILE)
                Logger.addDataReceiver(new WPILOGWriter("/AdvantageKit-ReplayLogs/Simulation/"));
            Logger.addDataReceiver(new NT4Publisher());
            break;
        case REPLAY:
            setUseTiming(false);
            String log_path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(log_path));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(log_path, "_sim")));
            break;
        default: Print.error("Invalid Robot Mode"); break;
        }
        LoggedPowerDistribution.getInstance(10, ModuleType.kRev);
        Logger.start();

        robot_container = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if(DriverStation.isDisabled()) enabled_flag = false;
        if(!enabled_flag && DriverStation.isEnabled()){
            enabled_flag = true;
            SubSystems.swerve_drive.postEnableSetup();
        }
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
        // IO.controller.setRumble(RumbleType.kLeftRumble, IO.controller.getRightTriggerAxis());
        // IO.controller.setRumble(RumbleType.kBothRumble, 1);
    }

    @Override public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override public void testPeriodic() {}

    @Override public void simulationInit() {}

    @Override public void simulationPeriodic() {}
}
