// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.LaunchNote;
// import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final SwerveDriveSubsystem swerve_drive_subsystem = new SwerveDriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    // swerve_drive_subsystem.r

    // swerve_drive_subsystem.setDefaultCommand(new SwerveJoystickCmd(
    //   swerve_drive_subsystem, 
    //   () -> -driver_controller.getLeftY(), // Y-Axis 
    //   () -> driver_controller.getLeftX(),  // X-Axis
    //   () -> driver_controller.getRightX(),  // Rot-Axis
    //   () -> !driver_controller.getHID().getAButtonPressed()  // Any button to set field orientation
    // ));
    // Configure the trigger bindings
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
    Constants.IO.commandController.a().whileTrue(shooterSubsystem.intakeCommand());
    Constants.IO.commandController.b().whileTrue(new LaunchNote(shooterSubsystem));


  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
