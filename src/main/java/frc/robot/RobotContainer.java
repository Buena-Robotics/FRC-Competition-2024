// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SubSystems;
import frc.robot.Constants.IO;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PathFindToClosestPose;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.RumbleFeedback;
import frc.robot.commands.ShooterTurret;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.climber.Climb.ArmPosition;
import frc.robot.subsystems.climber.ClimbReal;
import frc.robot.subsystems.climber.ClimbSim;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.notearm.NoteArm;
import frc.robot.subsystems.notearm.NoteArmReal;
import frc.robot.subsystems.notearm.NoteArmSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.utils.NoteVisualizer;
import frc.robot.utils.Print;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private final LoggedDashboardChooser<Command> auto_chooser;
    private boolean field_oriented_mode = false;

    public RobotContainer() {
        NamedCommands.registerCommand("retract_arm", new WaitCommand(2)); 
        NamedCommands.registerCommand("launch_note",
            new ParallelCommandGroup(
                new PrepareLaunch(SubSystems.shooter)
                    .withTimeout(1),
                SubSystems.climb.moveArmToPosition(ArmPosition.SPEAKER_CLOSE) )
                .andThen(new LaunchNote(SubSystems.shooter).withTimeout(0.25))
                .andThen(NoteVisualizer.shoot(SubSystems.swerve_drive::getPose, SubSystems.climb::getShooterAngleRadians)));
        NamedCommands.registerCommand("lift_note", SubSystems.note_arm.grabNoteFullCommand());
        NamedCommands.registerCommand("put_note", SubSystems.note_arm.releaseNoteFullCommand());

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> { Logger.recordOutput("PathPlanner/RobotPose", pose); });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> { Logger.recordOutput("PathPlanner/TargetPose", pose); });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            Pose2d[] target_path;
            target_path = poses.toArray(new Pose2d[poses.size()]);
            Logger.recordOutput("PathPlanner/TargetPath", target_path);
        });
        auto_chooser = new LoggedDashboardChooser<Command>("Auto Choices", AutoBuilder.buildAutoChooser("New Auto"));
        { // SYSID
            auto_chooser.addOption("Drive SysId (Quasistatic Forward)", SubSystems.swerve_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Quasistatic Reverse)", SubSystems.swerve_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            auto_chooser.addOption("Drive SysId (Dynamic Forward)", SubSystems.swerve_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Dynamic Reverse)", SubSystems.swerve_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        configureBindings();
    }

    private boolean turretMode(){
        final double distance = SubSystems.swerve_drive.getPose().getTranslation().getDistance(FieldConstants.getSpeakerPoint().getTranslation());
        return distance < 4 && RobotState.shooterHasNote() && false;
    }

    private Rotation2d getEstimatedShooterRotation(){
        final Transform3d robot_to_shooter = new Transform3d(new Translation3d(Units.inchesToMeters(-14.5),0.0,Units.inchesToMeters(21.5)), new Rotation3d());
        final Pose3d robot_pose = new Pose3d(SubSystems.swerve_drive.getPose());
        final Pose3d shooter_pose = robot_pose.plus(robot_to_shooter);
        
        final double speaker_height = Units.feetToMeters(6 + (8/12.0)) - Units.inchesToMeters(21.5);
        final double distance_to_speaker = shooter_pose.toPose2d().getTranslation().getDistance(FieldConstants.getSpeakerPoint().getTranslation());

        final double speaker_height_increase = distance_to_speaker < 1.5 ? 0 : distance_to_speaker / 8.0;

        final double estimated_shooter_pitch = Math.atan2(speaker_height + speaker_height_increase, distance_to_speaker);

        final Rotation2d estimated_rotation = new Rotation2d((Math.PI/2) - estimated_shooter_pitch);

        return estimated_rotation;
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
            () -> -IO.controller.getRightX(),  // Rot-Axis
            () -> field_oriented_mode, 
            () -> turretMode()
        ));
        SubSystems.climb.setDefaultCommand(SubSystems.climb.moveArmTriggers(this::getClimberSpeed));
        
        final Trigger turretTrigger = new Trigger(this::turretMode);
        turretTrigger.whileTrue(new ShooterTurret(SubSystems.climb, this::getEstimatedShooterRotation));

        IO.commandController.back().onTrue(new InstantCommand(() -> { field_oriented_mode = !field_oriented_mode; }));
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

        IO.shooterHasNoteTrigger.onTrue(new RumbleFeedback(IO.controller, RumbleType.kLeftRumble, 1, 500));
        IO.noteArmHasNoteTrigger.onTrue(new RumbleFeedback(IO.controller, RumbleType.kRightRumble, 1, 500));

        IO.commandController.start().onTrue(
            new PathFindToClosestPose().pathFindToClosestPose(SubSystems.swerve_drive, SubSystems.swerve_drive::getPose));
        IO.commandController.start().onFalse(Commands.runOnce(() -> {}, SubSystems.swerve_drive));
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
