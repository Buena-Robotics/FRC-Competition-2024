// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Controller {
    //Buttons
    public static final XboxController.Button A = XboxController.Button.kA
        ,B = XboxController.Button.kB
        ,X = XboxController.Button.kX
        ,Y = XboxController.Button.kY
        ,LB = XboxController.Button.kLeftBumper
        ,RB = XboxController.Button.kRightBumper
        ,LS = XboxController.Button.kLeftStick
        ,RS = XboxController.Button.kRightStick
        ,MENU = XboxController.Button.kStart
        ,NAVIGATION = XboxController.Button.kBack;
  //Axis
    public static final XboxController.Axis LS_Y = XboxController.Axis.kLeftY
        ,LS_X = XboxController.Axis.kLeftX
        ,RS_Y = XboxController.Axis.kRightY
        ,RS_X = XboxController.Axis.kRightX
        ,LT = XboxController.Axis.kLeftTrigger
        ,RT = XboxController.Axis.kRightTrigger;

    //Ports
    public static final int XBOXCONTROLLERPORT = 0;
  }

  public static class Drive {
    //Distance between right and left wheels
    public static final double TRACK_WIDTH = Units.inchesToMeters(-1); // 20?
    //Distance between front and back wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(-1); // 20?

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
    );

    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = -1;

    public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = -1;
    public static final double TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = -1;

    public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = -1;
    public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = -1;
  }
}
