// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.climber.ClimbReal;
import frc.robot.subsystems.climber.ClimbSim;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.notearm.NoteArm;
import frc.robot.subsystems.notearm.NoteArmReal;
import frc.robot.subsystems.notearm.NoteArmSim;
import frc.robot.subsystems.servo.ServoSub;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.CameraData;
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
        public static final Climb climb = Robot.isReal() ? new ClimbReal() : new ClimbSim();
        public static final Shooter shooter = Robot.isReal() ? new ShooterReal() : new ShooterSim();
        public static final NoteArm note_arm = Robot.isReal() ? new NoteArmReal() : new NoteArmSim();
        public static final SwerveDrive swerve_drive = new SwerveDrive();
        public static final ServoSub servo = new ServoSub();
        public static final Vision vision = new Vision(
                new CameraData("Microsoft_LifeCam_HD-3000 (1)", new Transform3d(
                        Units.inchesToMeters(14.5),
                        Units.inchesToMeters(14.5),
                        Units.inchesToMeters(69/4.0),
                        new Rotation3d(0,Units.degreesToRadians(-20), 0)))
                        );
    }
}