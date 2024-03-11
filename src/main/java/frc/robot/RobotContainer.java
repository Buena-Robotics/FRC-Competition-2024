// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SubSystems;
import frc.robot.Constants.IO;
import frc.robot.commands.DriveForTime;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PathFindToClosestPose;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.RumbleFeedback;
import frc.robot.commands.ShooterTurret;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.climber.Climb.ArmPosition;
import frc.robot.subsystems.climber.Climb;
import frc.robot.utils.FieldVisualizer;
import frc.robot.utils.NoteVisualizer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private final LoggedDashboardNumber delay_chooser;
    private final LoggedDashboardChooser<Command> auto_chooser;
    private boolean field_oriented_mode = false;

    public RobotContainer() {

        { // Named Commands
            NamedCommands.registerCommand("retract_arm", new WaitCommand(2)); 
            NamedCommands.registerCommand("launch_note", aimLaunchNoteReset());
            NamedCommands.registerCommand("lift_note", SubSystems.note_arm.grabNoteFullCommand());
            NamedCommands.registerCommand("put_note", SubSystems.note_arm.releaseNoteFullCommand());
        }

        { // PathPlanner Logging
            PathPlannerLogging.setLogCurrentPoseCallback((pose) -> { Logger.recordOutput("PathPlanner/RobotPose", pose); });
            PathPlannerLogging.setLogTargetPoseCallback((pose) -> { Logger.recordOutput("PathPlanner/TargetPose", pose); });
            PathPlannerLogging.setLogActivePathCallback((poses) -> {
                Pose2d[] target_path;
                target_path = poses.toArray(new Pose2d[poses.size()]);
                Logger.recordOutput("PathPlanner/TargetPath", target_path);
                FieldVisualizer.getField().getObject("Trajectory").setPoses(poses);
            });
        }
        
        auto_chooser = new LoggedDashboardChooser<Command>("Auto Choices", AutoBuilder.buildAutoChooser());
        delay_chooser = new LoggedDashboardNumber("Auto Delay", 0);

        if( !DriverStation.isFMSAttached() ) { // SYSID
            auto_chooser.addOption("Drive SysId (Quasistatic Forward)", SubSystems.swerve_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Quasistatic Reverse)", SubSystems.swerve_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            auto_chooser.addOption("Drive SysId (Dynamic Forward)", SubSystems.swerve_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Dynamic Reverse)", SubSystems.swerve_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
        { // Simple Autos
            auto_chooser.addDefaultOption("Simple Auto", emptyCommand());
        }

        configureBindings();
    }

    private boolean turretMode(){
        final double distance = SubSystems.swerve_drive.getPose().getTranslation().getDistance(FieldConstants.getSpeakerPoint().getTranslation());
        return distance < 3.5 && RobotState.shooterHasNote();
    }
    private Rotation2d getEstimatedShooterRotation(){
        final Transform3d robot_to_shooter = new Transform3d(new Translation3d(Units.inchesToMeters(-14.5),0.0,Units.inchesToMeters(21.5)), new Rotation3d());
        final Pose3d robot_pose = new Pose3d(SubSystems.swerve_drive.getPose());
        final Pose3d shooter_pose = robot_pose.plus(robot_to_shooter);
        
        final double distance_to_speaker = shooter_pose.toPose2d().getTranslation().getDistance(FieldConstants.getSpeakerPoint().getTranslation());

        final double bound = (distance_to_speaker - 1.646722) / (3.450181 - 1.646722);
        final Rotation2d estimated_rotation = Climb.ArmPosition.SPEAKER_CLOSE.getRotation().interpolate(Climb.ArmPosition.SPEAKER_STAGE.getRotation(), bound);  

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

        IO.commandController.rightBumper().whileTrue(launchNote());

        IO.commandController.a().onTrue(new InstantCommand(SubSystems.note_arm::openClaw));
        IO.commandController.b().onTrue(new InstantCommand(SubSystems.note_arm::closeClaw));

        IO.commandController.x().onTrue(SubSystems.note_arm.grabNoteFullCommand());
        IO.commandController.y().onTrue(SubSystems.note_arm.releaseNoteFullCommand());

        IO.commandController.povUp().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.UP));
        IO.commandController.povDown().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.SOURCE));
        IO.commandController.povLeft().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.SPEAKER_CLOSE));
        IO.commandController.povRight().onTrue(SubSystems.climb.moveArmToPosition(ArmPosition.SPEAKER_STAGE));

        IO.shooterHasNoteTrigger.debounce(0.4).onTrue(
            new RumbleFeedback(IO.controller, RumbleType.kLeftRumble, 1, 500)
                .alongWith(SubSystems.climb.moveArmToPosition(ArmPosition.DOWN)));
        IO.noteArmHasNoteTrigger.debounce(0.6).onTrue(new RumbleFeedback(IO.controller, RumbleType.kRightRumble, 1, 500));

        IO.commandController.start().onTrue(
            new PathFindToClosestPose().pathFindToClosestPose(SubSystems.swerve_drive, SubSystems.swerve_drive::getPose));
        IO.commandController.start().onFalse(Commands.runOnce(() -> {}, SubSystems.swerve_drive));
    }
    
    public Command emptyCommand(){
        Command empty = Commands.none();
        empty.setName("empty");
        return empty;
    }
    public Command launchNote(){
        return new PrepareLaunch(SubSystems.shooter)
            .withTimeout(1)
            .andThen(
                new LaunchNote(SubSystems.shooter).withTimeout(0.25),
                NoteVisualizer.shoot(SubSystems.swerve_drive::getPose, SubSystems.climb::getShooterAngleRadians)
            )
            .handleInterrupt(SubSystems.shooter::stop);
    }
    public Command aimLaunchNoteReset(){
        return SubSystems.climb.moveArmToPosition(ArmPosition.SPEAKER_CLOSE).andThen(launchNote(), new WaitCommand(0.2), SubSystems.climb.moveArmToPosition(ArmPosition.UP));
    }
    public Command simpleAuto(Rotation2d direction, long time_ms){
        return aimLaunchNoteReset().andThen(new DriveForTime(SubSystems.swerve_drive, direction, time_ms));
    }

    public Command getAutonomousCommand() {
        final Command auto_command = auto_chooser.get();
        final boolean use_simple_auto = emptyCommand().getName().equals(auto_command.getName());

        if(use_simple_auto) {
            final Pose2d robot_pose = SubSystems.swerve_drive.getPose();
            if(RobotState.isBlueAlliance())
                return simpleAuto(robot_pose.getRotation().unaryMinus(), 1000);
            else // Red Alliance
                return simpleAuto(robot_pose.getRotation().plus(Rotation2d.fromDegrees(180)).unaryMinus(), 1000);
        }

        return new WaitCommand(delay_chooser.get()).andThen(auto_command); 
    }
}
