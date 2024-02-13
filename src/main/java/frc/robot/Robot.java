// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Scanner;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  public static final PowerDistribution power_distribution = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
  private final RobotContainer robot_container = new RobotContainer();
  private Command autonomous_command;

  @Override public void robotInit() {
    final Scanner scanner = new Scanner(System.in);
    final String scanned_title = scanner.nextLine(); 
    scanner.close();

    final String date_pattern = "HH:mm:ss : dd-MM-yyyy";
    final SimpleDateFormat date_format = new SimpleDateFormat(date_pattern);
    final Calendar calender = Calendar.getInstance();

    final String metadata_date = date_format.format(calender.getTime());
    final String metadata_title = scanned_title != "" ? scanned_title : "Build - ";
    
    Logger.recordMetadata("Title", metadata_title); // Set a metadata value
    Logger.recordMetadata("Date", metadata_date); // Set a metadata value
    
    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start();
  }

  @Override public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override public void disabledInit() {}

  @Override public void disabledPeriodic() {}

  @Override public void autonomousInit() {
    autonomous_command = robot_container.getAutonomousCommand();

    if (autonomous_command != null) {
      autonomous_command.schedule();
    }
  }

  @Override public void autonomousPeriodic() {}

  @Override public void teleopInit() {
    if (autonomous_command != null) {
      autonomous_command.cancel();
    }
  }

  @Override public void teleopPeriodic() {}

  @Override public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override public void testPeriodic() {}

  @Override public void simulationInit() {}

  @Override public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
