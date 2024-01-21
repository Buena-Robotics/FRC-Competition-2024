// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.LaunchNote;
import frc.robot.commands.SwerveJoystickCmd;
// import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Constants.SubSystems.swerve_drive_subsystem.setDefaultCommand(new SwerveJoystickCmd(
      Constants.SubSystems.swerve_drive_subsystem, 
      () -> -Constants.IO.controller.getLeftY(), // Y-Axis 
      () -> Constants.IO.controller.getLeftX(),  // X-Axis
      () -> Constants.IO.controller.getRightX(),  // Rot-Axis
      () -> !Constants.IO.controller.getXButtonPressed()  // Any button to set field orientation
    ));
    Constants.IO.commandController.a().whileTrue(Constants.SubSystems.shooter_subsystem.intakeCommand());
    Constants.IO.commandController.b().whileTrue(new LaunchNote(Constants.SubSystems.shooter_subsystem));

    // Constants.IO.commandController.y().onPress(new InstantCommand(() -> System.out.println("press y")), 20);
    // Constants.IO.commandController.y().onRelease(new InstantCommand(() -> System.out.println("rel y")), 0);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
