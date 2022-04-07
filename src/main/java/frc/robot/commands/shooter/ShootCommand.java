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
  private enum State {
    SpinningUp,
    ShootingWrongCargo,
    ShootingCargo
  }

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  private final DoubleSupplier shootSupplier;
  private final DoubleSupplier quickShootSupplier;
  private final TurretCommand turretCommand; 

  private boolean startedShooting = false;

  private State currentState = State.SpinningUp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, DoubleSupplier shootSupplier, DoubleSupplier quickShootSupplier, TurretCommand turretCommand) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.shootSupplier = shootSupplier;
    this.quickShootSupplier = quickShootSupplier;
    this.turretCommand = turretCommand;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shoot Init");
    startedShooting = false;
    currentState = State.SpinningUp;
    shooter.setVision(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean actuallyShoot = shootSupplier.getAsDouble() > .8;
    boolean quickShoot = quickShootSupplier.getAsDouble() > .8;

    //System.out.printf("%b", actuallyShoot);
    boolean atSpeed = false;
    Boolean redCargo = indexer.isRedCargoAtPosition3();

    boolean wrongCargo = false;
    if(redCargo != null) {
      if(redCargo && DriverStation.getAlliance() == Alliance.Blue) {
        wrongCargo = true;
        
      }
      else if(!redCargo && DriverStation.getAlliance() == Alliance.Red) {
        wrongCargo = true;
      }
    }

    //atSpeed = shooter.shoot();

    //Distance is calculated from the edge of the right side of the robot to the front of the vision tape
    //atSpeed = shooter.shoot(.610, 38); //17 ft
    //atSpeed = shooter.shoot(.545, 21); //13 ft
    //atSpeed = shooter.shoot(.490, 0); //9 ft
    //atSpeed = shooter.shoot(.45, 0); //6 ft

    shooter.updateVisionData();

    boolean hasTarget = shooter.hasTarget();

    if(wrongCargo && !startedShooting) {
      currentState = State.ShootingWrongCargo;
      atSpeed = shooter.shoot(.35, 38);
    } else if((actuallyShoot || quickShoot) && !startedShooting) {
      currentState = State.ShootingCargo;
      atSpeed = shooter.shoot();
    }
    else if((!actuallyShoot && !quickShoot) && currentState != State.SpinningUp) {
      startedShooting = false;
      currentState = State.SpinningUp;
      shooter.spinUp();
    }


    if(atSpeed && !startedShooting) {
      startedShooting = true;
    }
    else if(hasTarget && startedShooting) {
      shooter.aquireTarget();
    }
    else if(!hasTarget) {
      turretCommand.moveTurret();
    }

    if(startedShooting || atSpeed) {
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
