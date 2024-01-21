// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.vendor.CXboxController;

public final class Constants {
  public static class IO {
    //Ports
    public static final int XBOXCONTROLLERPORT = 0;

    //Controllers
    public static final CXboxController commandController = new CXboxController(XBOXCONTROLLERPORT);

    public static final XboxController controller = new XboxController(XBOXCONTROLLERPORT);
  }

  public static class SubSystems{
      public static final SwerveDriveSubsystem swerve_drive_subsystem = new SwerveDriveSubsystem();
      public static final ShooterSubsystem shooter_subsystem = new ShooterSubsystem();
  }

  public static class Drive {
    //Swerve Drive L2
    //https://www.andymark.com/products/mk4-swerve-modules?Steering%20Encoder=CANCoder&quantity=1&Gear%20Ratio=L2%206.75%3A1&Includes%20Motors=No%20Motors&Motor%20Pinion%20Type=NEO&Assembled=No&Wheel%20Type=Billet%20Wheel

    //Distance between right and left wheels
    public static final double TRACK_WIDTH = Units.inchesToMeters(20); // 20?
    //Distance between front and back wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(20); // 20?

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
    );

    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14.5);

    public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3; //?
    public static final double TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3; //?

    public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = 5; // ?
    public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI; // ?
  }
}
