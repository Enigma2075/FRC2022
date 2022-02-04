// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbInCommand;
import frc.robot.commands.ClimbOutCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final ShootCommand shootCommand = new ShootCommand(shooterSubsystem);
  private final ClimbOutCommand climbOutCommand = new ClimbOutCommand(climberSubsystem);
  private final ClimbInCommand climbInCommand = new ClimbInCommand(climberSubsystem);

  XboxController driverController =  new XboxController(IOConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    climberSubsystem.setDefaultCommand(new ClimbCommand(climberSubsystem, driverController::getLeftTriggerAxis, driverController::getRightTriggerAxis));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kA.value)
      .whenHeld(shootCommand);

    new JoystickButton(driverController, Button.kB.value)
      .whenHeld(climbInCommand);

    new JoystickButton(driverController, Button.kX.value)
      .whenHeld(climbOutCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return shootCommand;
  }
}
