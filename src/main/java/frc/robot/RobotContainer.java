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

public class RobotContainer {

  public RobotContainer() {
    configureBindings();
  }

  private double getRotationSpeed(){
    if(IO.controller.getPOV() == 270) // LEFT
      return -0.09;
    else if(IO.controller.getPOV() == 90) // RIGHT
      return 0.09;
    return IO.controller.getRightX();
  }

  private double getForwardSpeed(){
    if(IO.controller.getPOV() == 0) // Forward
      return -0.09;
    else if(IO.controller.getPOV() == 180) // Backward
      return 0.09;
    return IO.controller.getLeftY();
  }

  private double getSideSpeed(){
    return IO.controller.getLeftX();
  }

  private double getClimberSpeed() {
    if (IO.commandController.getLeftCTriggerAxis() > 0.01) 
      return -IO.commandController.getLeftCTriggerAxis();

    return IO.commandController.getRightCTriggerAxis();
  }

  private boolean isShooting() {
    return !IO.controller.getXButton();
  }

  // private void showTrajectory(){
  //   Vector<Pose2d> trajectory = DriveToFieldPosCmd.getTrajectory(FieldPoses.ROBOT_BLUE_AMP);
  //   SubSystems.swerve_drive_subsystem.getField2d().getObject("trajectory").setPoses(trajectory);;
  // }

  private void configureBindings() {
    SubSystems.swerve_drive_subsystem.setDefaultCommand(new SwerveJoystickCmd(
      SubSystems.swerve_drive_subsystem, 
      this::getForwardSpeed, // Y-Axis 
      this::getSideSpeed,  // X-Axis
      this::getRotationSpeed  // Rot-Axis
    ));
    IO.commandController.leftBumper().whileTrue(SubSystems.shooter_subsystem.intakeCommand());
    IO.commandController.rightBumper().whileTrue(new PrepareLaunch(SubSystems.shooter_subsystem)
                                                      .withTimeout(0.5)
                                                      .andThen(new LaunchNote(SubSystems.shooter_subsystem))
                                                      .handleInterrupt(SubSystems.shooter_subsystem::stop));
    // IO.commandController.a().onTrue(new InstantCommand(this::showTrajectory));
    // IO.commandController.x().onTrue(new DriveToFieldPosCmd(SubSystems.swerve_drive_subsystem, FieldPoses.ROBOT_BLUE_AMP));
    // IO.commandController.y().toggleOnTrue(new InstantCommand(SubSystems.swerve_drive_subsystem::toggleAprilTags));
    IO.commandController.a().onTrue(SubSystems.note_arm_subsystem.grabNoteFullCommand());
    IO.commandController.b().onTrue(SubSystems.note_arm_subsystem.releaseNoteFullCommand());

    IO.commandController.rightCTrigger().whileTrue(new MoveArmCommand(SubSystems.climb_subsystem, getClimberSpeed(), isShooting()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
