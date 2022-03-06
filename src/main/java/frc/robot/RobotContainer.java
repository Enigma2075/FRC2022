// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.climber.StartClimb;
import frc.robot.commands.ResetPivot;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.auto.GrabCargo;
import frc.robot.commands.auto.RightFull;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.commands.climber.LatchHigh;
import frc.robot.commands.climber.MoveClimbCommand;
import frc.robot.commands.climber.Pullup;
import frc.robot.commands.climber.PullupHigh;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootNotCommand;
import frc.robot.commands.shooter.TurretCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberArmSubsystem.PivotPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  XboxController driverController =  new XboxController(IOConstants.kDriverControllerPort);
  XboxController operatorController =  new XboxController(IOConstants.kOperatorControllerPort);

  // The robot's subsystems and commands are defined here...
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(gyroSubsystem);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  
  private final RightFull grabCargoCommand = new RightFull(gyroSubsystem, driveSubsystem, shooterSubsystem, indexerSubsystem, intakeSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    climberSubsystem.setDefaultCommand(new ClimbCommand(climberSubsystem, operatorController::getLeftY));
    
    intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem, climberSubsystem, driverController::getRightTriggerAxis, driverController::getLeftTriggerAxis));

    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driverController::getLeftY, driverController::getRightX, driverController::getLeftBumper));

    indexerSubsystem.setDefaultCommand(new IndexerCommand(indexerSubsystem));

    shooterSubsystem.setDefaultCommand(new TurretCommand(shooterSubsystem, gyroSubsystem, operatorController::getPOV, operatorController::getLeftX, operatorController::getLeftY));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(operatorController, Button.kRightBumper.value)
      .whenHeld(new ShootCommand(shooterSubsystem, indexerSubsystem, operatorController::getLeftTriggerAxis));

    new JoystickButton(operatorController, Button.kLeftBumper.value)
      .whenHeld(new ShootNotCommand(shooterSubsystem, indexerSubsystem));

    new JoystickButton(operatorController, Button.kB.value)
      .whenHeld(new Pullup(climberSubsystem));

    new JoystickButton(operatorController, Button.kX.value)
      .whenHeld(new StartClimb(climberSubsystem));
    
    new JoystickButton(operatorController, Button.kY.value)
      .whenHeld(new PullupHigh(climberSubsystem, driverController::getBButton));

    new JoystickButton(operatorController, Button.kA.value)
      .whenHeld(new LatchHigh(climberSubsystem));

    new JoystickButton(driverController, Button.kA.value)
      .whenHeld(new ResetPivot(climberSubsystem, intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return grabCargoCommand;
  }

  public void teleopInit() {
    driveSubsystem.setDriveMode(DriveMode.Normal);
    driveSubsystem.setNeutralMode(NeutralMode.Coast);
    shooterSubsystem.setLEDs(false);
  }

  public void updateDrive() {
    //System.out.println("UpdateDrive");
    driveSubsystem.update();
  }
}
