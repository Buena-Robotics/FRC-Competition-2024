// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CenterSourceCmd;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.SwerveJoystickCmd;
// import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public RobotContainer() {
    configureBindings();
  }

  private double getRotationSpeed(){
    if(Constants.IO.controller.getPOV() == 270) // LEFT
      return 0.09;
    else if(Constants.IO.controller.getPOV() == 90) // RIGHT
      return -0.09;
    return -Constants.IO.controller.getRightX();
  }

  private double getForwardSpeed(){
    if(Constants.IO.controller.getPOV() == 0) // Forward
      return 0.09;
    else if(Constants.IO.controller.getPOV() == 180) // Backward
      return -0.09;
    return -Constants.IO.controller.getLeftY();
  }

  private double getSideSpeed(){
    return -Constants.IO.controller.getLeftX();
  }

  private void configureBindings() {
    Constants.SubSystems.swerve_drive_subsystem.setDefaultCommand(new SwerveJoystickCmd(
      Constants.SubSystems.swerve_drive_subsystem, 
      this::getForwardSpeed, // Y-Axis 
      this::getSideSpeed,  // X-Axis
      this::getRotationSpeed  // Rot-Axis
      // () -> !Constants.IO.controller.getXButtonPressed()  // Any button to set field orientation
    ));
    Constants.IO.commandController.leftBumper().whileTrue(Constants.SubSystems.shooter_subsystem.intakeCommand());
    Constants.IO.commandController.rightBumper().whileTrue(new PrepareLaunch(Constants.SubSystems.shooter_subsystem)
                                                              .withTimeout(1.0)
                                                              .andThen(new LaunchNote(Constants.SubSystems.shooter_subsystem))
                                                              .handleInterrupt(Constants.SubSystems.shooter_subsystem::stop));
    Constants.IO.commandController.x().onTrue(new CenterSourceCmd(Constants.SubSystems.swerve_drive_subsystem));
    Constants.IO.commandController.y().toggleOnTrue(new InstantCommand(() -> Constants.SubSystems.swerve_drive_subsystem.toggleAprilTags()));
    // Constants.IO.commandController.y().onTrue(new InstantCommand(Constants.SubSystems.swerve_drive_subsystem::zeroHeading));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
