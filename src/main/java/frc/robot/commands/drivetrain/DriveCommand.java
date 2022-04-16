// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.Shoot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;
  private final DoubleSupplier throttle;
  private final DoubleSupplier wheel;
  private final BooleanSupplier fastMode;
  private final XboxController driverController;
  private final XboxController operatorController;

  private boolean isRumbling = false;
  private boolean isRumblingOperator = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem drive, DoubleSupplier throttle, DoubleSupplier wheel, BooleanSupplier fastMode, XboxController driverController, XboxController operatorController) {
    this.drive = drive;
    this.throttle = throttle;
    this.wheel = wheel;
    this.fastMode = fastMode;
    this.driverController = driverController;
    this.operatorController = operatorController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public double applyDeadband(double joystick){
    double deadband = .11;

    if(Math.abs(joystick) <= deadband) {
      return 0;
    }
    else {
      double remappedJoystick = (joystick - Math.signum(joystick) * deadband) / (1.0 - deadband);
      if(!fastMode.getAsBoolean()) {
        remappedJoystick *= .6;
      }
 
      return remappedJoystick;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttleValue = throttle.getAsDouble() * -1;
    double wheelValue = wheel.getAsDouble();
    
    double adjustedThrottle = applyDeadband(throttleValue);
    double adjustedWheel = applyDeadband(wheelValue);

    if(ClimberSubsystem.hasClimbStarted()) {
      drive.driveSlow(adjustedThrottle, adjustedWheel);
    }
    else if (ShooterSubsystem.isShooting() && IndexerSubsystem.hasCargo()) {
      drive.driveSlow(adjustedThrottle, adjustedWheel);
    }
    else {
      drive.drive(adjustedThrottle, adjustedWheel);
    }
    
    //SmartDashboard.putBoolean("Shooter:StuckOnPost", ShooterSubsystem.isStuckOnPost());
      
    if((ShooterSubsystem.isTargetFound() || !ShooterSubsystem.isSpinningUp()) && isRumblingOperator) {
      isRumblingOperator = false;
      operatorController.setRumble(RumbleType.kLeftRumble, 0);
    }
    else if(!ShooterSubsystem.isTargetFound() && !isRumblingOperator && ShooterSubsystem.isSpinningUp()) {
      isRumblingOperator = true;
      operatorController.setRumble(RumbleType.kLeftRumble, 1);
    }
    
    if(ShooterSubsystem.isStuckOnPost() && !isRumbling && ShooterSubsystem.isSpinningUp()) {
      isRumbling = true;
      driverController.setRumble(RumbleType.kLeftRumble, 1);
    }
    else if((!ShooterSubsystem.isStuckOnPost() || !ShooterSubsystem.isSpinningUp()) && isRumbling) {
      isRumbling = false;
      driverController.setRumble(RumbleType.kLeftRumble, 0);
    }

    //SmartDashboard.putNumber("Drive Command:Wheel", adjustedWheel);
    //SmartDashboard.putNumber("Drive Command:throttle", adjustedThrottle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isRumbling = false;
    driverController.setRumble(RumbleType.kLeftRumble, 0);   
  }
}