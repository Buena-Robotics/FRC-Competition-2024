// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final SwerveDriveSubsystem swerve_drive_subsystem = new SwerveDriveSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve_drive_subsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerve_drive_subsystem, 
      () -> -Constants.IO.controller.getLeftY(), // Y-Axis 
      () -> Constants.IO.controller.getLeftX(),  // X-Axis
      () -> Constants.IO.controller.getRightX(),  // Rot-Axis
      () -> !Constants.IO.controller.getAButtonPressed()  // Any button to set field orientation
    ));

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
