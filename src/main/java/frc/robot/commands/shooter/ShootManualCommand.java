// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShootManualCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final double targetAngle;
  private final double speed;
  private final double hoodPosition;

  private boolean turretAtPosition = false;
  private boolean shooting = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootManualCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, double targetAngle, double speed,
      double hoodPosition) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.targetAngle = targetAngle;
    this.speed = speed;
    this.hoodPosition = hoodPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setVision(false);
    shooter.turret(targetAngle);

    shooting = false;
    turretAtPosition = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean atSpeed = false;
    if ((Math.abs(targetAngle - shooter.getTurretAngle()) < 5 || turretAtPosition) && !shooting) {
      turretAtPosition = true;

      atSpeed = shooter.shoot(speed, hoodPosition); // 114.02 Distance
    }

    if (atSpeed || shooting) {
      shooting = true;
      indexer.index(true);
    } else {
      indexer.index();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVision(false);
    shooter.stop();
    indexer.stop();
  }
}
