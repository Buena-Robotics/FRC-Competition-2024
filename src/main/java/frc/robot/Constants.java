// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.climber.ClimbReal;
import frc.robot.subsystems.climber.ClimbSim;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.notearm.NoteArm;
import frc.robot.subsystems.notearm.NoteArmReal;
import frc.robot.subsystems.notearm.NoteArmSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.CameraData;
import frc.robot.subsystems.vision.VisionCamera.PhotonPipeline;
import frc.robot.vendor.CXboxController;

public final class Constants {
    public static class IO {
        //Ports
        public static final int XBOXCONTROLLERPORT = 0;
        //Controllers
        public static final CXboxController commandController = new CXboxController(XBOXCONTROLLERPORT);
        public static final XboxController controller = new XboxController(XBOXCONTROLLERPORT);

        public static final Trigger shooterHasNoteTrigger = new Trigger(() -> RobotState.shooterHasNote());
        public static final Trigger noteArmHasNoteTrigger = new Trigger(() -> RobotState.armHasNote());
    }

    public static class SubSystems{
        public static final Climb climb = Robot.isReal() ? new ClimbReal() : new ClimbSim();
        public static final Shooter shooter = Robot.isReal() ? new ShooterReal() : new ShooterSim();
        public static final NoteArm note_arm = Robot.isReal() ? new NoteArmReal() : new NoteArmSim();
        public static final SwerveDrive swerve_drive = new SwerveDrive();
        public static final Vision vision = new Vision(
                new CameraData("Microsoft_LifeCam_HD-3000 (1) (2)", new Transform3d(
                        Units.inchesToMeters(12.5),
                        Units.inchesToMeters(13),
                        Units.inchesToMeters(18),
                        new Rotation3d(0,Units.degreesToRadians(-38), Units.degreesToRadians(0))), PhotonPipeline.APRILTAG),
                new CameraData("USB_Camera", new Transform3d(
                        Units.inchesToMeters(-6),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(42.5),
                        new Rotation3d(0,Units.degreesToRadians(40), 0)), PhotonPipeline.NOTE_DETECTION)
                        );
                        //6inches back
    }

    public static class Empty {
        public static final Rotation3d R3D_ZERO = new Rotation3d();
        public static final Rotation2d R2D_ZERO = new Rotation2d();
        public static final Translation2d TRANSL2D_ZERO = new Translation2d();
    }
}