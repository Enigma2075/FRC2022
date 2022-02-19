// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;
  private final DoubleSupplier throttle;
  private final DoubleSupplier wheel;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem drive, DoubleSupplier throttle, DoubleSupplier wheel) {
    this.drive = drive;
    this.throttle = throttle;
    this.wheel = wheel;

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
      return (joystick - Math.signum(joystick) * deadband) / (1.0 - deadband);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttleValue = throttle.getAsDouble() * -1;
    double wheelValue = wheel.getAsDouble();
    
    double adjustedThrottle = applyDeadband(throttleValue);
    double adjustedWheel = applyDeadband(wheelValue);

    drive.drive(adjustedThrottle, adjustedWheel);
    
    SmartDashboard.putNumber("Drive Command:Wheel", adjustedWheel);
    SmartDashboard.putNumber("Drive Command:throttle", adjustedThrottle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
}
