// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IO;
import frc.robot.Constants.SubSystems;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.SwerveJoystickCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    public RobotContainer() {
        configureBindings();
    }

    private double getRotationSpeed(){
        if(IO.controller.getPOV() == 270) // LEFT
            return 0.09;
        else if(IO.controller.getPOV() == 90) // RIGHT
            return -0.09;
        return -IO.controller.getRightX();
    }

    private double getForwardSpeed(){
        if(IO.controller.getPOV() == 0) // Forward
            return -0.09;
        else if(IO.controller.getPOV() == 180) // Backward
            return 0.09;
        return -IO.controller.getLeftY();
    }

    private double getSideSpeed(){
        return -IO.controller.getLeftX();
    }

    private double getClimberSpeed() {
        if (IO.controller.getLeftTriggerAxis() > 0.01) 
            return -IO.controller.getLeftTriggerAxis();
        return IO.controller.getRightTriggerAxis();
    }

    private boolean isShooting() {
        return !IO.controller.getXButton();
    }

    // private void showTrajectory(){
    //   Vector<Pose2d> trajectory = DriveToFieldPosCmd.getTrajectory(FieldPoses.ROBOT_BLUE_AMP);
    //   SubSystems.swerve_drive_subsystem.getField2d().getObject("trajectory").setPoses(trajectory);;
    // }

    private void configureBindings() {
        SubSystems.swerve_drive.setDefaultCommand(new SwerveJoystickCmd(
            SubSystems.swerve_drive, 
            this::getForwardSpeed, // Y-Axis 
            this::getSideSpeed,  // X-Axis
            this::getRotationSpeed  // Rot-Axis
        ));
        SubSystems.climb_subsystem.setDefaultCommand(new MoveArmCommand(SubSystems.climb_subsystem, this::getClimberSpeed, isShooting()));
        IO.commandController.leftBumper().whileTrue(SubSystems.shooter.intakeCommand());
        IO.commandController.rightBumper().whileTrue(new PrepareLaunch(SubSystems.shooter)
                                                        .withTimeout(0.5)
                                                        .andThen(new LaunchNote(SubSystems.shooter))
                                                        .handleInterrupt(SubSystems.shooter::stop));
        // IO.commandController.a().onTrue(new InstantCommand(this::showTrajectory));
        // IO.commandController.x().onTrue(new DriveToFieldPosCmd(SubSystems.swerve_drive_subsystem, FieldPoses.ROBOT_BLUE_AMP));
        // IO.commandController.y().toggleOnTrue(new InstantCommand(SubSystems.swerve_drive_subsystem::toggleAprilTags));
        IO.commandController.a().onTrue(new InstantCommand(SubSystems.note_arm::openClaw));
        IO.commandController.b().onTrue(new InstantCommand(SubSystems.note_arm::closeClaw));

        IO.commandController.x().onTrue(SubSystems.note_arm.grabNoteFullCommand());
        IO.commandController.y().onTrue(SubSystems.note_arm.releaseNoteFullCommand());

        // IO.commandController.rightCTrigger().whileTrue(new MoveArmCommand(SubSystems.climb_subsystem, getClimberSpeed(), isShooting()));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
