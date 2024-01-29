// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static class FieldPoses {
    public static final double NOTE_SIZE    = Units.inchesToMeters(13);
    public static final double ROBOT_LENGTH = Units.inchesToMeters(26);
    public static final double ROBOT_WIDTH  = Units.inchesToMeters(26);

    public static final Pose2d
      ROBOT_BLUE_AMP = new Pose2d(1.84, 7.78, new Rotation2d(Units.degreesToRadians(90)));
      // blue_source_inner = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(-60))),
      // blue_source_center = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(-60))),
      // blue_source_outer = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(-60))),
      // blue_stage_shot = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
      // blue_hang_left = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
      // blue_hang_center = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
      // blue_hang_right = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
      // blue_speaker_left = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
      // blue_speaker_center = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
      // blue_speaker_right = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
      // red_amp = new Pose2d(14.7, 7.78, new Rotation2d(Units.degreesToRadians(90)));,
    public static final Pose2d 
      NOTE_BLUE_TOP             = new Pose2d(2.887, 7.010, new Rotation2d()),
      NOTE_BLUE_CENTER          = new Pose2d(2.887, 5.560, new Rotation2d()),
      NOTE_BLUE_BOTTOM          = new Pose2d(2.887, 4.108, new Rotation2d()),
      NOTE_CENTER_TOP           = new Pose2d(8.270, 7.470, new Rotation2d()),
      NOTE_CENTER_CENTER_TOP    = new Pose2d(8.270, 5.790, new Rotation2d()),
      NOTE_CENTER_CENTER        = new Pose2d(8.270, 4.108, new Rotation2d()),
      NOTE_CENTER_CENTER_BOTTOM = new Pose2d(8.270, 2.430, new Rotation2d()),
      NOTE_CENTER_BOTTOM        = new Pose2d(8.270, 0.750, new Rotation2d()),
      NOTE_RED_TOP              = new Pose2d(13.65, 7.010, new Rotation2d()),
      NOTE_RED_CENTER           = new Pose2d(13.65, 5.560, new Rotation2d()),
      NOTE_RED_BOTTOM           = new Pose2d(13.65, 4.108, new Rotation2d());
  }

  public static class Drive {
    //Swerve Drive L2
    //https://www.andymark.com/products/mk4-swerve-modules?Steering%20Encoder=CANCoder&quantity=1&Gear%20Ratio=L2%206.75%3A1&Includes%20Motors=No%20Motors&Motor%20Pinion%20Type=NEO&Assembled=No&Wheel%20Type=Billet%20Wheel

    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14);

    public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 0.5; //?
    public static final double TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 0.5; //?

    public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = 1; // ?
    public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // ?
  }
}
