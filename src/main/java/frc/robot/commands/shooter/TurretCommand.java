// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class TurretCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter;
  private final DoubleSupplier headingSupplier;
  private final DoubleSupplier headingX;
  private final DoubleSupplier headingY;
  private final GyroSubsystem gyro;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretCommand(ShooterSubsystem shooter, GyroSubsystem gyro, DoubleSupplier headingSupplier, DoubleSupplier headingX, DoubleSupplier headingY) {
    this.shooter = shooter;
    this.headingSupplier = headingSupplier;
    this.headingX = headingX;
    this.headingY = headingY;
    this.gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.setHood();

    double requestedHeading = -1;
    
    //if(headingY.getAsDouble() > .2 && headingX.getAsDouble() >.2) {
    //  requestedHeading = Math.atan2(headingY.getAsDouble(), headingX.getAsDouble()) - Math.PI / 4;
    //}
        
    requestedHeading = headingSupplier.getAsDouble();
    if(requestedHeading == -1) {
      shooter.turret(20);
      return;
    }

    requestedHeading = 360 - requestedHeading;

    double currentYaw = gyro.getYaw() % 360.0;

    if(Math.signum(currentYaw) == -1) {
      currentYaw += 360.0;
    }
    
    double finalHeading = requestedHeading - currentYaw;
    if(Math.signum(finalHeading) == -1) {
      finalHeading += 360.0;
    }

    //SmartDashboard.putNumber("RequestedYaw", requestedHeading);
    //SmartDashboard.putNumber("Yaw", gyro.getYaw());
    //SmartDashboard.putNumber("CalculatedYaw", currentYaw);    
    //SmartDashboard.putNumber("FinalHeading", finalHeading);    
    
    shooter.turret(finalHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
