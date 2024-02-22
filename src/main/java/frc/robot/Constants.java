// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.notearm.NoteArm;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
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
        public static final ClimbSubsystem climb_subsystem = new ClimbSubsystem();
        public static final Vision vision = Robot.isReal() ? new VisionReal() : new VisionSim();
        public static final NoteArm note_arm = new NoteArm();
        public static final SwerveDrive swerve_drive = new SwerveDrive();
        public static final ShooterSubsystem shooter = new ShooterSubsystem();
    }

    public static class FieldPoses {
        public static final Pose2d
            ROBOT_BLUE_AMP = new Pose2d(1.79, 8.06, new Rotation2d(Units.degreesToRadians(90)));
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
    }
}