// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IO;
import frc.robot.Constants.SubSystems;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.climber.Climb.ArmPosition;
import frc.robot.utils.NoteVisualizer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

    private final LoggedDashboardChooser<Command> auto_chooser;

    public RobotContainer() {
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> { Logger.recordOutput("PathPlanner/RobotPose", pose); });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> { Logger.recordOutput("PathPlanner/TargetPose", pose); });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            Pose2d[] target_path;
            target_path = poses.toArray(new Pose2d[poses.size()]);
            Logger.recordOutput("PathPlanner/TargetPath", target_path);
        });

        auto_chooser = new LoggedDashboardChooser<Command>("Auto Choices");
        { // SYSID
            auto_chooser.addOption("Drive SysId (Quasistatic Forward)", SubSystems.swerve_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Quasistatic Reverse)", SubSystems.swerve_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            auto_chooser.addOption("Drive SysId (Dynamic Forward)", SubSystems.swerve_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Dynamic Reverse)", SubSystems.swerve_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        configureBindings();
    }

    private double getClimberSpeed() {
        if (IO.controller.getLeftTriggerAxis() > 0.01) 
            return -IO.controller.getLeftTriggerAxis();
        return IO.controller.getRightTriggerAxis();
    }

    private void configureBindings() {
        SubSystems.swerve_drive.setDefaultCommand(new SwerveJoystick(
            SubSystems.swerve_drive, 
            () -> -IO.controller.getLeftY(), // Y-Axis 
            () -> -IO.controller.getLeftX(),  // X-Axis
            () -> -IO.controller.getRightX()  // Rot-Axis
        ));
        SubSystems.climb.setDefaultCommand(SubSystems.climb.moveArmTriggers(this::getClimberSpeed));
        IO.commandController.leftBumper().whileTrue(SubSystems.shooter.intakeCommand());

        IO.commandController.rightBumper().whileTrue(NoteVisualizer.shoot(SubSystems.swerve_drive::getPose, SubSystems.climb::getShooterAngleRadians)
                                                        .alongWith(
                                                            new PrepareLaunch(SubSystems.shooter)
                                                            .withTimeout(1)
                                                            .andThen(new LaunchNote(SubSystems.shooter))
                                                            .handleInterrupt(SubSystems.shooter::stop)));

        IO.commandController.a().onTrue(new InstantCommand(SubSystems.note_arm::openClaw));
        IO.commandController.b().onTrue(new InstantCommand(SubSystems.note_arm::closeClaw));

        IO.commandController.x().onTrue(SubSystems.note_arm.grabNoteFullCommand());
        IO.commandController.y().onTrue(SubSystems.note_arm.releaseNoteFullCommand());

        IO.commandController.povUp().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.UP));
        IO.commandController.povDown().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.SOURCE));
        IO.commandController.povLeft().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.SPEAKER_CLOSE));
        IO.commandController.povRight().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.SPEAKER_STAGE));
    }

    public Command getAutonomousCommand() {
        return auto_chooser.get();
        // PathConstraints constraints = new PathConstraints(
        //         3.0, 4.0,
        //         Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(
        //     FieldConstants.getSpeakerCenterPathfindPose(),
        //     constraints,
        //     0.0, // Goal end velocity in meters/sec
        //     0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        // );

        // return pathfindingCommand;
    }
}
