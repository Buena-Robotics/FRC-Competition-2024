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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    public RobotContainer() {
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
            () -> IO.controller.getLeftY(), // Y-Axis 
            () -> IO.controller.getLeftX(),  // X-Axis
            () -> IO.controller.getRightX()  // Rot-Axis
        ));
        SubSystems.climb.setDefaultCommand(SubSystems.climb.moveArmTriggers(this::getClimberSpeed));
        IO.commandController.leftBumper().whileTrue(SubSystems.shooter.intakeCommand());

        IO.commandController.rightBumper().whileTrue(new PrepareLaunch(SubSystems.shooter)
                                                        .withTimeout(1)
                                                        .andThen(new LaunchNote(SubSystems.shooter))
                                                        .handleInterrupt(SubSystems.shooter::stop));
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
        return SubSystems.climb.moveArmToPosition(ArmPosition.SPEAKER_CLOSE).andThen(
            new PrepareLaunch(SubSystems.shooter).withTimeout(1)
                                                    .andThen(new LaunchNote(SubSystems.shooter))
                                                    .handleInterrupt(SubSystems.shooter::stop));
        // final double kPXController = 2;
        // final double kPYController = 2;
        // final double kPThetaController = 3;

        //  final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        //         new TrapezoidProfile.Constraints(
        //                 Math.PI / 3,
        //                 Math.PI / 6);

        // // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         Units.feetToMeters(15.1),
        //         Units.feetToMeters(4))
        //                 .setKinematics(SubSystems.swerve_drive.getKinematics());

        // // 2. Generate trajectory
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         SubSystems.swerve_drive.getPose(),
        //         List.of(
        //                 new Translation2d(2, 2),
        //                 new Translation2d(3, 6),
        //                 new Translation2d(-1,-3)),
        //         SubSystems.swerve_drive.getPose(),
        //         trajectoryConfig);
        // FieldVisualizer.setTrajectory(trajectory);
        // System.out.println(trajectory.getTotalTimeSeconds());
        // // 3. Define PID controllers for tracking trajectory
        // PIDController xController = new PIDController(kPXController, 0, 0);
        // PIDController yController = new PIDController(kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         kPThetaController, 0, 0, kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // 4. Construct command to follow trajectory
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         trajectory,
        //         SubSystems.swerve_drive::getPose,
        //         SubSystems.swerve_drive.getKinematics(),
        //         xController,
        //         yController,
        //         thetaController,
        //         SubSystems.swerve_drive::setModuleStates,
        //         SubSystems.swerve_drive);

        // // 5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(
        //         // new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        //         swerveControllerCommand,
        //         new InstantCommand(() -> SubSystems.swerve_drive.xStopModules()));
    }
}
