// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.aquireTarget();
    //boolean atSpeed = shooter.shoot(.48);
    
    double slope = (.53 - .42)/(114.02 - 53.2);

    double speed = slope * shooter.getDistanceFromTarget() + (.53 - (114.02 * slope));

    
    //boolean atSpeed = shooter.shoot(.42); // 43.5 Distance
    //boolean atSpeed = shooter.shoot(.53); // 114.02 Distance

    boolean atSpeed = shooter.shoot(speed); // 114.02 Distance
    
    SmartDashboard.putNumber("Shooter:Distance", shooter.getDistanceFromTarget());

    if(atSpeed) {
      indexer.index(true);
    }
    else {
      indexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }
}
