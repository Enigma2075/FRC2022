// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class TurretCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter;
  private final DoubleSupplier headingXSupplier;
  private final DoubleSupplier headingYSupplier;
  private final GyroSubsystem gyro;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretCommand(ShooterSubsystem shooter, GyroSubsystem gyro, DoubleSupplier headingXSupplier, DoubleSupplier headingYSupplier) {
    this.shooter = shooter;
    this.headingXSupplier = headingXSupplier;
    this.headingYSupplier = headingYSupplier;
    this.gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setLEDs(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ClimberSubsystem.hasClimbStarted()) {
      shooter.turret(180);
      return;
    }
    //shooter.setHood();

    double requestedHeading = -1;
    
    double headingX = headingXSupplier.getAsDouble();
    double headingY = headingYSupplier.getAsDouble() * -1;

    if(Math.abs(headingX) > .3 || Math.abs(headingY) > .3) {
      requestedHeading = Math.toDegrees(Math.atan2(headingY, headingX));  
    }

    if(requestedHeading == -1) {
      shooter.turret(20);
      return;
    }

    // Invert the heading as the dpad moves clockwise.
    requestedHeading = (requestedHeading - 90);
    if(requestedHeading < 0) {
      requestedHeading = 360 + requestedHeading;
    }

    double currentYaw = (gyro.getYaw() - 90) % 360.0;

    if(Math.signum(currentYaw) == -1) {
      currentYaw += 360.0;
    }
    
    double finalHeading = requestedHeading - currentYaw;

    if(Math.signum(finalHeading) == -1) {
      finalHeading += 360.0;
    }

    //SmartDashboard.putNumber("RequestedHeading", requestedHeading);
    //SmartDashboard.putNumber("Yaw", gyro.getYaw());
    //SmartDashboard.putNumber("CalculatedYaw", currentYaw);    
    //SmartDashboard.putNumber("FinalHeading", finalHeading);    
    //SmartDashboard.putNumber("RequestedHeading", requestedHeading);    
    
    shooter.turret(finalHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
