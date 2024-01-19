// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.ControllerUtils;
// import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDriveSubsystem swerve_drive_subsystem = new SwerveDriveSubsystem();

  private final CommandXboxController driver_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve_drive_subsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerve_drive_subsystem, 
      () -> -driver_controller.getLeftY(), // Y-Axis 
      () -> driver_controller.getLeftX(),  // X-Axis
      () -> driver_controller.getRightX(),  // Rot-Axis
      () -> !driver_controller.getHID().getAButtonPressed()  // Any button to set field orientation
    ));
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // new JoystickButton(driver_controller.getHID(), 2).onTrue(() -> swerve_drive_subsystem.zeroHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
