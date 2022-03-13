// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  private final DoubleSupplier shootSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, DoubleSupplier shootSupplier) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.shootSupplier = shootSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter.showVision(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //shooter.shoot(true);

    boolean actuallyShoot = shootSupplier.getAsDouble() > .8;

    //System.out.printf("%b", actuallyShoot);
    boolean atSpeed = false;
    Boolean redCargo = indexer.isRedCargoAtPosition3();

    boolean wrongCargo = false;
    //if(redCargo != null) {
    //  if(redCargo && DriverStation.getAlliance() != Alliance.Red) {
    //    wrongCargo = true;
    //  }
    //  else if(!redCargo && DriverStation.getAlliance() != Alliance.Blue) {
    //    wrongCargo = true;
    //  }
    //}

    if(wrongCargo) {
      atSpeed = shooter.shoot(.25);
    } else if(actuallyShoot) {
      atSpeed = shooter.shoot(); // 114.02 Distance
    }
    else {
      shooter.spinUp();
    }
    
    if(atSpeed) {
      indexer.index(true);
    }
    else {
      indexer.index();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.showVision(false);
    shooter.stop();
    indexer.stop();
  }
}
